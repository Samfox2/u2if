import time
import threading
import hid
import serial
from . import helper
from . import u2if_const as report_const

COMPATIBLE_BOARD_PID_VID = [
    # (VID, PID)
    (0xcafe, 0x4005),  # pico
    (0x239a, 0x00f1),  # Adafruit Feather
    (0x239a, 0x8130),  # Adafruit Feather CAN Bus
    (0x239a, 0x812C),  # Adafruit Feather ThinkInk
    (0x239a, 0x812E),  # Adafruit Feather RFM
    (0x239a, 0x00fd),  # Adafruit ItsyBitsy
    (0x239a, 0x0105),  # Adafruit KB2040
    (0x239a, 0x0109),  # Adafruit QT2040 Trinket
    (0x239a, 0x00f7),  # Adafruit QTPY
]


class Device(metaclass=helper.Singleton):

    def __init__(self, serial_number_str=None):
        self.vid, self.pid, self.serial_number = self._get_compatible_board_and_reset(serial_number_str)
        if self.serial_number is None:
            raise ValueError("No board found")

        time.sleep(1)

        self._hid = hid.Device(self.vid, self.pid, self.serial_number)
        device = helper.find_serial_port(self.vid, self.pid, self.serial_number)
        self._serial = serial.Serial(device)

        self._irq_event_callbacks = {}

        self._response_lock = threading.Lock()
        self._pending_responses = {}
        self._pending_conditions = {}
        self._running = True

        self._listener_thread = threading.Thread(
            target=self._hid_listener_loop,
            name="u2if-hid-listener",
            daemon=True,
        )
        self._listener_thread.start()

    self.firmware_version = self._get_firmware_version()

    def close(self):
        self._running = False
        self._listener_thread.join(timeout=1)
        self._hid.close()
        try:
            self._hid.close()
        except Exception:
            pass

    def _reset(self):
        res = self.send_report(bytes([report_const.SYS_RESET]), response=True)
        if res[1] != report_const.OK:
            raise RuntimeError("Reset error.")

    def _get_compatible_board_and_reset(self, serial_number_str=None):
        for vid, pid in COMPATIBLE_BOARD_PID_VID:
            try:
                self._hid = hid.Device(vid, pid, serial_number_str)
                serial_number = self._hid.serial
                self._reset()
                self._hid.close()
            except hid.HIDException:
                continue
            return vid, pid, serial_number
        return None, None, None

    def get_serial_number(self):
        return self.serial_number

    def get_firmware_version(self):
        return self.firmware_version

    def _hid_listener_loop(self):
        """
        Block on HID reads and dispatch either:
        - synchronous command responses
        - asynchronous GPIO IRQ events
        """
        while self._running:
            try:
                res = self._hid.read(report_const.HID_REPORT_SIZE)
                if not res:
                    continue

                report_id = res[0]

                # async IRQ handling
                if report_id == report_const.GPIO_ASYNC_IRQ:
                    self._dispatch_irq_event(res)
                    continue

                # normal response to send_report()
                with self._response_lock:
                    self._pending_responses[report_id] = res
                    cond = self._pending_conditions.get(report_id)
                    if cond is not None:
                        cond.notify_all()

            except Exception:
                # optional: logging
                time.sleep(0.05)

    def _dispatch_irq_event(self, res):
        """
        Firmware format:
            res[0] = GPIO_ASYNC_IRQ
            res[1] = status
            res[2] = count
            res[3..] = packed events

        packed event:
            lower 6 bits = gpio
            upper 2 bits = event (RISING/FALLING)
        """

        if len(res) < 3:
            return

        status = res[1]
        if status != report_const.OK:
            return

        irq_nb = res[2]

        for irq_index in range(3, 3 + irq_nb):
            ev_key = res[irq_index]

            gpio = ev_key & 0b00111111
            event = (ev_key >> 6) & 0b00000011

            callback = self._irq_event_callbacks.get(gpio)
            if callback is not None:
                try:
                    callback(gpio, event=event)
                except Exception:
                    # optional: log error
                    pass

    def send_report(self, report, response=True, timeout=2.0):
        report_id = report[0]

        if response:
            with self._response_lock:
                if report_id not in self._pending_conditions:
                    self._pending_conditions[report_id] = threading.Condition(
                        self._response_lock
                    )
                cond = self._pending_conditions[report_id]

                self._pending_responses.pop(report_id, None)

                self._hid.write(
                    b"\0"
                    + report
                    + b"\0" * (report_const.HID_REPORT_SIZE - len(report))
                )

                end_time = time.monotonic() + timeout
                while report_id not in self._pending_responses:
                    remaining = end_time - time.monotonic()
                    if remaining <= 0:
                        raise TimeoutError(f"Timeout waiting for response 0x{report_id:02X}")
                    cond.wait(timeout=remaining)

                res = self._pending_responses.pop(report_id)

            if res[1] == report_const.NOT_CONCERNED:
                raise RuntimeError(
                    "Unknown command. Maybe the interface is not enabled in firmware."
                )

            return res

        self._hid.write(
            b"\0" + report + b"\0" * (report_const.HID_REPORT_SIZE - len(report))
        )
        return None

    def read_hid(self, report_id):
        res = self._hid.read(report_const.HID_REPORT_SIZE)
        while res[0] != report_id:
            # self._report_events_list.append(res)
            res = self._hid.read(report_const.HID_REPORT_SIZE)
        return res

    def reset_output_serial(self):
        self._serial.reset_output_buffer()

    def write_serial(self, buf):
        self._serial.write(buf)
        if len(buf) % report_const.HID_REPORT_SIZE == 0:
            # Strange behavior of the CDC, if one suddenly sends a buffer of size multiple of
            # 64 (size of the USB report), the last BULK transfer is not received or remains pending.
            # We add a character, it is not a problem, it will be discared in the protocol.
            self._serial.write([0])
        self._serial.flush()

    def process_irq(self):
        res = self.send_report(bytes([report_const.GPIO_GET_IRQ]))
        if res[1] != report_const.OK:
            raise RuntimeError("IRQ retrieve error.")
        irq_nb = res[2]
        for irq_index in range(3, 3 + irq_nb):
            ev_key = res[irq_index]
            gpio = ev_key & 0b111111
            event = (ev_key >> 6) & 0b11
            if gpio in self._irq_event_callbacks:
                self._irq_event_callbacks[gpio](gpio, event=event)

    def register_callback(self, gpio, callback):
        self._irq_event_callbacks[gpio] = callback

    def unregister_callback(self, gpio):
        if gpio in self._irq_event_callbacks:
            del self._irq_event_callbacks[gpio]

    # def _get_serial_number(self):
    #     response = self.send_report(bytes([report_const.SYS_GET_SN]))
    #     if response[1] != report_const.OK:
    #         raise RuntimeError("Retrieve S/N error.")
    #     sn = "0x"
    #     for i in range(2,2+8):
    #         sn += "{0:02X}".format(response[i])
    #     return sn

    def _get_firmware_version(self):
        response = self.send_report(bytes([report_const.SYS_GET_VN]))
        if response[1] != report_const.OK:
            raise RuntimeError("Retrieve V/N error.")
        else:
            return list(response[2:2+3])
