# Author:   TV
# Purpose:  Run avrdude directly to burn bootloader and set fuses 
#           as pio fails to do so automatically (P.I.T.A, can't inherit specified .ini params properly)
#           specifcally made for Uno-as-ISP programmer (running ArduinoISP.cpp, adjusted for pio)
# Note on Programmer Protocol Selection:
# protocol `arduinoisp` = stk500v1 with custom config (config alias), unreliable. Does not work in pio.
# Explicitly using stk500v1 along with the port and baud rate for a direct, 
# reliable command to bypass param inheritance and auto-detection induced issues in pio.

Import("env")

def burn_fuses_and_bootloader(source, target, env):
    # Use env.subst() to get the correct upload port from the environment
    upload_port = env.subst("$UPLOAD_PORT")
    if not upload_port:
        print("Error: Could not determine upload port. Please specify it in platformio.ini.")
        env.Exit(1)

    print("Using upload port: " + upload_port)

    # All env.Execute() calls now use the resolved upload_port
    # 3.3V/8MHz fuse settings for internal 8MHz oscillator
    env.Execute("avrdude -C/etc/avrdude.conf -c stk500v1 -p m328p -P " + upload_port + " -b 19200 -U lfuse:w:0xE2:m -U hfuse:w:0xDA:m -U efuse:w:0x05:m")
    env.Execute("avrdude -C/etc/avrdude.conf -c stk500v1 -p m328p -P " + upload_port + " -b 19200 -U flash:w:" + env.subst("$PROJECT_PACKAGES_DIR/framework-arduino-avr/bootloaders/optiboot/optiboot_atmega328.hex") + ":i")
    env.Execute("avrdude -C/etc/avrdude.conf -c stk500v1 -p m328p -P " + upload_port + " -b 19200 -U lock:w:0x0F:m")

# Register the custom target
env.AddCustomTarget(
    name="burn-bootloader-custom",
    dependencies=[],
    actions=burn_fuses_and_bootloader,
    title="Burn Bootloader"
)

# Attach the script to a pre-upload action
env.AddPreAction("upload", burn_fuses_and_bootloader)