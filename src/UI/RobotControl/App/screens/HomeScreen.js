import React from 'react';
import { TextInput, TouchableHighlight, Image, Alert, Button, StyleSheet, Text, View } from 'react-native';
function onPressLearnMore() {
  Alert.alert(
    'Sample Alert',
    'Sample Alert',
    [
      {text: 'Ask me later', onPress: () => console.log('Ask me later pressed')},
      {text: 'Cancel', onPress: () => console.log('Cancel Pressed'), style: 'cancel'},
      {text: 'OK', onPress: () => console.log('OK Pressed')},
    ],
    { cancelable: false }
  )
}

export default class HomeScreen extends React.Component {
  constructor(props) {
    super(props);
    this.state = {currentPage: 'Loading'};
  }
  render(){
    return (
      <View style={styles.topContainer}>
        <View style={styles.container}>
          <View style={styles.imageContainer}>
            <Image
              source={require('../static/iiit_logo.gif')}
              style={{width: 130, height: 84}}
            />
          </View>
          <Button
            onPress={() => {this.props.navigation.push('SendMessage')}}
            title="Send Message"
            accessibilityLabel="Comes to the current location and then takes the message to specified location"
          />
          <Button
            onPress={onPressLearnMore}
            title="Start Tour Guide"
            accessibilityLabel="Starts a tour of the area guided by the robot"
          />
          <Button
            onPress={() => {this.props.navigation.push('CurrentStatus')}}
            title="Current Status"
            accessibilityLabel="Shows info about the current status of the bot"
          />
          <Button
            onPress={() => {this.props.navigation.push('Chat')}}
            title="Chat"
            accessibilityLabel="Chat with the bot"
          />
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
