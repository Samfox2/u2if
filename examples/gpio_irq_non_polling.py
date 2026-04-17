#!/usr/bin/env python3

"""
Example: Non-polling GPIO interrupt handling using u2if

This example demonstrates how to use asynchronous GPIO interrupts
without calling Pin.process_irq() in a polling loop.
"""

import time
from machine import u2if, Pin


# --- Callback function ---
def on_button_event(pin, event=None):
    """
    This function is called automatically when an IRQ occurs.
    """
    if event == Pin.IRQ_FALLING:
        print(f"[IRQ] Falling edge detected on GPIO {pin}")
    elif event == Pin.IRQ_RISING:
        print(f"[IRQ] Rising edge detected on GPIO {pin}")
    else:
        print(f"[IRQ] Event {event} on GPIO {pin}")


def main():
    print("Starting non-polling GPIO IRQ example...")

    # Configure pin (example: GP8 with pull-up)
    button = Pin(u2if.GP8, Pin.IN, pull=Pin.PULL_UP)

    # Register interrupt callback
    button.irq(
        handler=on_button_event,
        trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING,
        debounce=True,
    )

    print("Waiting for interrupts... (press Ctrl+C to exit)")

    try:
        # No polling loop required!
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nExiting...")


if __name__ == "__main__":
    main()
