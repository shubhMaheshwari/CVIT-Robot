/**
 * Sample React Native App
 * https://github.com/facebook/react-native
 * @flow
 */

import React, { Component } from 'react';
import {
  Platform,
  StyleSheet,
  Text,
  View,
  Image,
  Button
} from 'react-native';



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
          <Text style={styles.welcome}>
            Select a time limit on your tour.
            </Text>
          <Button
            onPress={() => {this.props.navigation.push('Story',{content_duration:'1',client:client = this.props.navigation.state.params.client.client})}}
            title="30 min"
            color="#841584"
            accessibilityLabel="Comes to the current location and then takes the message to specified location"
          />
          <Button
            onPress={() => {this.props.navigation.push('Story',{content_duration:'2',client:client = this.props.navigation.state.params.client.client})}}
            title="45 min"
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
