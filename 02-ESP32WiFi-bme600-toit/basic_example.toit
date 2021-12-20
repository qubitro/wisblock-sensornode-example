import qubitro
import bme680
import i2c
import gpio

main:
    led_pin := gpio.Pin 12 --output
    led_pin.set 1
    bus := i2c.Bus
        --sda=gpio.Pin 4
        --scl=gpio.Pin 5
    sdevice := bus.device bme680.I2C_ADDRESS
    sensor := bme680.Driver sdevice
    
    client ::= qubitro.connect
        --id="DEVICE_ID"
        --token="DEVICE_TOKEN"

    led_pin.set 0

    while true:
        led_pin.set 1
        client.publish{"temp": "$(%.1f sensor.read_temperature)", "hum": "$(%.1f sensor.read_humidity)", "pressure": "$(%.1f sensor.read_pressure / 100)", "gas":"$(%.3f sensor.read_gas / 1000)"}
        print "Published message to Qubitro!"
        led_pin.set 0
        sleep --ms=10000
    
    led_pin.close
