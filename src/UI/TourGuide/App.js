import React,{Component} from 'react';
import { ScrollView, TextInput, TouchableHighlight, Image, Alert, Button, StyleSheet, Text, View } from 'react-native';
import {createStackNavigator} from 'react-navigation'


// import Sound from 'react-native-sound';
import {AudioRecorder, AudioUtils} from 'react-native-audio';

import Location from './screens/Location';
import Welcome from './screens/Welcome';
import Story from './screens/Story';
import Home from './screens/Home';


import EventEmitter2 from 'eventemitter2'
window.EventEmitter2 = EventEmitter2.EventEmitter2
var RosClient = require("roslibjs-client");
var client = new RosClient({
  url: "ws://192.168.43.137:9090"
});
client.on("connected", function() {
  console.log("Connection established!");
  // client.topic.publish('/speaker', 'std_msgs/String', {'data':'welcome to kcis'})
  // var listener = client.topic.subscribe('/speaker', "std_msgs/String", (message) => {
  //   console.log(message)
  // });
});
client.on("disconnected", function() {
  console.log("Connection disconnected!");
});



export default class App extends React.Component {
  constructor(props) {
    super(props);
    
    var ws = new WebSocket('ws://10.42.0.1:9999');

    ws.onopen = () => {
      ws.send('something'); // send a message
    };

    ws.onmessage = (e) => {
      console.log(e.data);
    };
  }
  render() {
      return (
        <AppStackNavigator/>
   
      );

    
  }
}

const AppStackNavigator = createStackNavigator({
  Welcome : Welcome,
  Home :  Home,
  Location : Location,
  Story: Story
})

const styles = StyleSheet.create({
  container: {
    backgroundColor: '#fff',
    height: '70%',
    width: '70%',
    alignItems: 'stretch',
    justifyContent: 'space-around',
  },
  topContainer: {
    backgroundColor: '#fff',
    height: '100%',
    width: '100%',
    alignItems: 'center',
    justifyContent: 'center',
  },
  pageHorizontalContainer: {
    backgroundColor: '#fff',
    height: '100%',
    width: '100%',
    flexDirection: 'row',
    alignItems: 'flex-start',
    justifyContent: 'flex-start',
  },
  topPageContainer: {
    backgroundColor: '#fff',
    height: '100%',
    width: '100%',
    alignItems: 'center',
    justifyContent: 'center',
  },
  pageContainer: {
    backgroundColor: '#fff',
    height: '100%',
    width: '100%',
    alignItems: 'flex-start',
    justifyContent: 'flex-start',
  },
  imageContainer: {
    backgroundColor: '#fff',
    width: '100%',
    alignItems: 'center',
    justifyContent: 'center',
  },
  welcome: {
    fontSize: 20,
    textAlign: 'center',
    margin: 10,
  },
  backContainer: {
    backgroundColor: '#fff',
    height: '15%',
    alignItems: 'center',
    justifyContent: 'center',
  },
});
