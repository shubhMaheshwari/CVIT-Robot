import React from 'react';
import { TextInput, Image, Button, StyleSheet, View } from 'react-native';
import EventEmitter2 from 'eventemitter2'
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

export default class Login extends React.Component {
  constructor(props) {
    super(props);
    this.state = {text: null};
  }
  onSubmitEdit = () => {
    client.topic.publish('/user_input', 'std_msgs/String', {'data':this.state.text});
  };
  render() {
    return (
      <View style={styles.topContainer}>
        <View style={styles.container}>
          <View style={styles.imageContainer}>
            <Image
              source={require('../static/iiit_logo.gif')}
              style={{width: 130, height: 84}}
            />
            <TextInput
              style={styles.input}
              textAlign="center"
              onSubmitEditing={this.onSubmitEdit}
              onChangeText={(text) => this.setState({text})}
            />
            <Button
              onPress={this.onSubmitEdit}
              title="submit"
              color="#841584"
            />
          </View>
        </View>
      </View>
    );
  }
}


const styles = StyleSheet.create({
  container: {
    backgroundColor: '#fff',
    height: '70%',
    width: '70%',
    alignItems: 'stretch',
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
    height: '70%',
    width: '70%',
    alignItems: 'flex-start',
    justifyContent: 'flex-start',
  },
  imageContainer: {
    backgroundColor: '#fff',
    width: '100%',
    alignItems: 'center',
    justifyContent: 'center',
  },
  backContainer: {
    backgroundColor: '#fff',
    height: '15%',
    alignItems: 'center',
    justifyContent: 'center',
  },
});
