import React from 'react';
import { StyleSheet, Text, View ,Button } from 'react-native';
import {createStackNavigator} from 'react-navigation'
import EventEmitter2 from 'eventemitter2'

import HomeScreen from './screens/HomeScreen'
import Location from './screens/Location'
import Chat from './screens/Chat'
import Status from './screens/Status'
import Login from './screens/Login'

window.EventEmitter2 = EventEmitter2.EventEmitter2
var RosClient = require("roslibjs-client");
var client = new RosClient({
  url: "ws://10.2.132.18:9090"
});

client.on("connected", function() {
  console.log("Connection established!");
});
client.on("disconnected", function() {
  console.log("Connection disconnected!");
});

export default class App extends React.Component {
  render() {
    return (
      <AppStackNavigator/>
    );
  }
}

const AppStackNavigator = createStackNavigator({
  Login: Login,
  Home :  HomeScreen,
  SendMessage : Location,
  Chat : Chat,
  CurrentStatus: Status
})

const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: '#fff',
    alignItems: 'center',
    justifyContent: 'center',
  },
});
