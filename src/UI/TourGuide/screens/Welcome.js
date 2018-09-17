/**
 * Sample React Native App
 * https://github.com/facebook/react-native
 * @flow
 */

import React, { Component } from 'react';
import { ScrollView, TextInput, TouchableHighlight, Image, Alert, Button, StyleSheet, Text, View } from 'react-native';

// import Sound from 'react-native-sound';
import {AudioRecorder, AudioUtils} from 'react-native-audio';

import EventEmitter2 from 'eventemitter2'
window.EventEmitter2 = EventEmitter2.EventEmitter2
var RosClient = require("roslibjs-client");
var client = new RosClient({
  url: "ws://10.2.131.218:9090"
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

// For audio recording 
let audioPath = AudioUtils.DocumentDirectoryPath + '/test.aac';

AudioRecorder.prepareRecordingAtPath(audioPath, {
  SampleRate: 44100,
  Channels: 1,
  AudioQuality: "Low",
  AudioEncoding: "aac"
});


type Props = {};
export default class Welcome extends Component<Props> {
  render() {
    return (
     
        <View style={styles.container}>
        <View style={styles.imageContainer}>
              <Image
                source={require('../images/logo.png')}
                style={{width: 130, height: 84}}
              />
            </View>
      <View style={styles.container}>
        <Text style={styles.welcome}>
          Welcome to Tour Guide
        </Text>
        <Text style={styles.instructions}>
          To get started, Press START!!
        </Text>
        
      </View>
      <Button
          onPress={() => {
            client.topic.publish('/speaker', 'std_msgs/String', {'data':'Starting the tour'});
            console.log("Connection ");
            // var listener = client.topic.subscribe('/speaker', "std_msgs/String", (message) => {
            //   console.log(message)
            // });
            this.props.navigation.push('Home',{client:{client}});
            }}
          title="START!"
          color="#841584"
          accessibilityLabel="Comes to the current location and then takes the message to specified location"
        />
  
       
      </View>
    );
  }
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
    justifyContent: 'center',
    alignItems: 'center',
    backgroundColor: '#F5FCFF',
  },
  welcome: {
    fontSize: 20,
    textAlign: 'center',
    margin: 10,
  },
  instructions: {
    textAlign: 'center',
    color: '#333333',
    marginBottom: 5,
  },
});
