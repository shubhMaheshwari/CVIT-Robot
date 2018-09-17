import React from 'react';
// import React from 'react';
import { Image, Alert, Button, StyleSheet, Text, View, Picker } from 'react-native';

export default class Location extends React.Component {
  constructor(props) {
    super(props);
    this.state = {location: 'location1'};
  }
  render() {
    return (
      <Picker
        selectedValue={this.state.location}
        style={{ height: 50, width: '80%' }}
        onValueChange={(itemValue, itemIndex) => this.setState({location: itemValue})}>
        <Picker.Item label="IRE" value="IRE" />
        <Picker.Item label="MT and NLP" value="location2" />
        <Picker.Item label="Speech processing" value="location3" />
        <Picker.Item label="CVIT" value="location4" />
        <Picker.Item label="MLL" value="location5" />
        <Picker.Item label="Cognitive science" value="location6" />
        <Picker.Item label="DSAC" value="location7" />
      </Picker>
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
  imageContainer: {
    backgroundColor: '#fff',
    width: '100%',
    alignItems: 'center',
    justifyContent: 'center',
  },
});
