import React from 'react';
import { TextInput, Image, Alert, Button, StyleSheet, Text, View } from 'react-native';

import Expo from 'expo';


export default class Login extends React.Component {
  constructor(props) {
    super(props);
    this.state = {name: ''};
  }
  componentWillMount() {
    Expo.SecureStore.getItemAsync('name').then((name) => {
      if(name !== null) {
        this.props.navigation.push('Home');
      }
    });
  }

  render() {
    return (
      <View style={styles.topContainer}>
        <View style={styles.container}>
          <View style={styles.imageContainer}>
            <Image
              source={require('../static/iiit_logo.gif')}
              style={{width: 130, height: 84}}
            />
          </View>
          <TextInput
            style={{height: 40, padding: 5}}
            onChangeText={(name) => this.setState({name})}
            onSubmitEditing={() => {
              this.props.navigation.push('Home');
              Expo.SecureStore.setItemAsync('name', this.state.name);
            }}
            value={this.state.name}
            placeholder="Enter Your Name"
          />
          <Button
            onPress={() => {
              this.props.navigation.push('Home');
              Expo.SecureStore.setItemAsync('name', this.state.name);
            }}
            title="OK"
            color="#841584"
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
