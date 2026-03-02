package main

import (
	"fmt"

	"github.com/lxzan/event_emitter"
	"github.com/lxzan/gws"
)

type Subscriber gws.Conn

func NewSubscriber(conn *gws.Conn) *Subscriber { return (*Subscriber)(conn) }

func (c *Subscriber) GetSubscriberID() int64 {
	userId, _ := c.GetMetadata().Load("userId")
	return userId.(int64)
}

func (c *Subscriber) GetMetadata() event_emitter.Metadata { return c.Conn().Session() }

func (c *Subscriber) Conn() *gws.Conn { return (*gws.Conn)(c) }

func Subscribe(em *event_emitter.EventEmitter[int64, *Subscriber], s *Subscriber, topic string) {
	em.Subscribe(s, topic, func(msg any) {
		_ = msg.(*gws.Broadcaster).Broadcast(s.Conn())
	})
}

func Publish(em *event_emitter.EventEmitter[int64, *Subscriber], topic string, msg []byte) {
	var broadcaster = gws.NewBroadcaster(gws.OpcodeText, msg)
	defer broadcaster.Close()
	em.Publish(topic, broadcaster)
}

type wsHandler struct {
	em      *event_emitter.EventEmitter[int64, *Subscriber]
	ch      chan []byte
	userID  int64
	topic   string
}

func (c *wsHandler) RunPublishLoop() {
	var msg []byte
	for {
		msg = <-c.ch
		Publish(c.em, c.topic, msg)
	}
}

func (c *wsHandler) OnOpen(socket *gws.Conn) {
	socket.Session().Store("userId", c.userID)
	c.userID++
	subscriber := NewSubscriber(socket)
	Subscribe(c.em, subscriber, c.topic)
}

func (c *wsHandler) OnClose(socket *gws.Conn, err error) {
	subscriber := NewSubscriber(socket)
	c.em.UnSubscribe(subscriber, c.topic)
}

func (c *wsHandler) OnMessage(socket *gws.Conn, message *gws.Message) {
	defer message.Close()
	fmt.Println(string(message.Bytes()))
	Publish(c.em, c.topic, message.Bytes())
}

func (c *wsHandler) OnPing(socket *gws.Conn, payload []byte) {}

func (c *wsHandler) OnPong(socket *gws.Conn, payload []byte) {}
