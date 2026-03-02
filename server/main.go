/*
* Server script which acts as a BLE central device
 */

package main

import (
	"net/http"

	"github.com/go-chi/chi/v5"
	"github.com/go-chi/chi/v5/middleware"
	"github.com/lxzan/event_emitter"
	"github.com/lxzan/gws"
)

const (
	DeviceName         = "Nano33BLE"
	ServiceUUID        = "88aadeff-64a4-47ae-8798-7d7e51b24e55"
	CharacteristicUUID = "88aadeff-64a4-47ae-8798-7d7e51b24e56"
)

func main() {
	ch := make(chan []byte, 10)
	bleCentral := NewBLECentral(DeviceName, ServiceUUID, CharacteristicUUID, ch)

	go bleCentral.Connect()

	em := event_emitter.New[int64, *Subscriber](&event_emitter.Config{
		BucketNum:  16,
		BucketSize: 128,
	})
	handler := &wsHandler{em: em, ch: ch, userID: 0, topic: "ble-data"}

	upgrader := gws.NewUpgrader(handler, &gws.ServerOption{
		ParallelEnabled:   true,
		Recovery:          gws.Recovery,
		PermessageDeflate: gws.PermessageDeflate{Enabled: true},
	})

	go handler.RunPublishLoop()

	r := chi.NewRouter()
	r.Use(middleware.Logger)

	r.Get("/ws", func(w http.ResponseWriter, r *http.Request) {
		socket, err := upgrader.Upgrade(w, r)
		if err != nil {
			return
		}
		go func() {
			socket.ReadLoop()
		}()
	})

	http.ListenAndServe(":3000", r)
}
