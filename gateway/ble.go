package main

import (
	"fmt"
	"log"

	"tinygo.org/x/bluetooth"
)

type message struct {
	Result       map[string]float32 `json:"result"`
	Measurements map[string]float32 `json:"measurements"`
}

type BLECentral struct {
	deviceName         string
	serviceUUID        *bluetooth.UUID
	characteristicUUID *bluetooth.UUID
	ch                 chan []byte
	adapter            *bluetooth.Adapter
}

func NewBLECentral(deviceName string, sUUID string, cUUID string, ch chan []byte) *BLECentral {
	central := &BLECentral{deviceName: deviceName, serviceUUID: &bluetooth.UUID{}, characteristicUUID: &bluetooth.UUID{}, ch: ch, adapter: bluetooth.DefaultAdapter}

	central.serviceUUID.UnmarshalText([]byte(sUUID))
	central.characteristicUUID.UnmarshalText([]byte(cUUID))

	return central
}

func (c *BLECentral) Connect() {
	c.adapter.Enable()

	var target bluetooth.ScanResult
	c.adapter.Scan(func(adapter *bluetooth.Adapter, result bluetooth.ScanResult) {
		fmt.Println("found device:", result.Address.String(), result.RSSI, result.LocalName())
		if result.LocalName() == c.deviceName {
			target = result
			adapter.StopScan()
		}
	})

	device, err := c.adapter.Connect(target.Address, bluetooth.ConnectionParams{})
	if err != nil {
		log.Fatalf("error connecting to target device: %s", err)
	}

	fmt.Println("established connection with target device")
	services, err := device.DiscoverServices([]bluetooth.UUID{*c.serviceUUID})
	if err != nil {
		log.Fatalf("error discovering services: %s", err)
	}

	if len(services) < 1 {
		log.Fatalf("could not discover service on target device")
	}

	service := services[0]
	chrcs, err := service.DiscoverCharacteristics([]bluetooth.UUID{*c.characteristicUUID})
	if err != nil {
		log.Fatalf("error discovering characteristics: %s", err)
	}

	if len(chrcs) < 1 {
		log.Fatalf("could not discover characteristic on target service and device")
	}

	chrc := chrcs[0]
	chrc.EnableNotifications(func(buf []byte) {
		fmt.Printf("%d\n", buf)
		c.ch <- buf
	})
}
