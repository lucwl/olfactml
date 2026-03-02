/*
* Server script which acts as a BLE central device
 */

package main

import (
	"fmt"
	"log"

	"tinygo.org/x/bluetooth"
)

var (
	serviceUUID        = &bluetooth.UUID{}
	characteristicUUID = &bluetooth.UUID{}
	targetName         = "Nano33BLE"
)

func init() {
	serviceUUID.UnmarshalText([]byte("88aadeff-64a4-47ae-8798-7d7e51b24e55"))
	characteristicUUID.UnmarshalText([]byte("88aadeff-64a4-47ae-8798-7d7e51b24e56"))
}

var adapter = bluetooth.DefaultAdapter

func main() {
	// Enable BLE interface.
	must("enable BLE stack", adapter.Enable())

	var target bluetooth.ScanResult
	adapter.Scan(func(adapter *bluetooth.Adapter, result bluetooth.ScanResult) {
		fmt.Println("found device:", result.Address.String(), result.RSSI, result.LocalName())
		if result.LocalName() == "Nano33BLE" {
			target = result
			adapter.StopScan()
		}
	})

	device, err := adapter.Connect(target.Address, bluetooth.ConnectionParams{})
	if err != nil {
		log.Fatalf("error connecting to target device: %s", err)
	}

	fmt.Println("established connection with target device")
	services, err := device.DiscoverServices([]bluetooth.UUID{*serviceUUID})
	if err != nil {
		log.Fatalf("error discovering services: %s", err)
	}

	if len(services) < 1 {
		log.Fatalf("could not discover service on target device")
	}

	service := services[0]
	chrcs, err := service.DiscoverCharacteristics([]bluetooth.UUID{*characteristicUUID})
	if err != nil {
		log.Fatalf("error discovering characteristics: %s", err)
	}

	if len(chrcs) < 1 {
		log.Fatalf("could not discover characteristic on target service and device")
	}

	chrc := chrcs[0]
	chrc.EnableNotifications(func(buf []byte) {
		fmt.Printf("received: %v\n", buf[0])
	})

	select {}
}

func must(action string, err error) {
	if err != nil {
		panic("failed to " + action + ": " + err.Error())
	}
}
