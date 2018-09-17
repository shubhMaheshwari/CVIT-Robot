import React, { Component } from 'react';
// import React from 'react';
import {
  Platform,
  StyleSheet,
  Text,
  View,
  Button,
  TextInput,
  ScrollView
} from 'react-native';
// import EventEmitter2 from 'eventemitter2'

import EventEmitter2 from 'eventemitter2'
window.EventEmitter2 = EventEmitter2.EventEmitter2
var RosClient = require("roslibjs-client");
var client = new RosClient({
  url: "ws://192.168.43.137:9090"
});


export default class Story extends React.Component{
  constructor(props) {
    super(props);
    this.state={
      labStory : '',
      labName: '',
      next_location_id: 1,
      curent_location_id: -1,
      direction: ''
    }
  }

  componentDidMount(){

    var listener = client.topic.subscribe('/tour_guide_data', "std_msgs/String", (message) => {
        message = JSON.parse(message.data)
        console.log(message)

        this.setState({
          labStory: message.story,
          labName: message.name,
          direction: message.direction
        })
      });


    client.on("disconnected", function() {
      console.log("Connection disconnected!");
    });
    
  }

  nextPage = () => {
    client.topic.publish('/tour_guide', 'std_msgs/String', {'data':"DIRECTION$"+(this.state.curent_location_id + 1)})

    this.setState({next_location_id: this.state.curent_location_id + 1 , curent_location_id: -1});  
  };
  onSubmitEdit = () => {

    this.setState({next_location_id: -1 , curent_location_id: this.state.next_location_id});
    console.log(this.props.navigation.state.params.content_duration);
    client.topic.publish('/tour_guide', 'std_msgs/String', {'data':"STORY$"+this.state.next_location_id+'$'+this.props.navigation.state.params.content_duration})

};

  render() {
    console.log("Next place",this.state.next_location_id )
    console.log("Next place",this.state.curent_location_id )
    if(this.state.next_location_id == -1){
      return (
        <View style={styles.container}>
          <View>
          <Text style={styles.welcome}>
          reached: {this.state.labName}
          </Text>
          <Text style={styles.instructions}>
            Story: {this.state.labStory ? this.state.labStory : "Loading"}
          </Text>
          <Button
            onPress={() => {this.nextPage()}}
            title="Go to next location"
            color="#841584"
            accessibilityLabel="Comes to the current location and tells the story"
          />
          </View>
        </View>
      );
	}

    else if( this.state.next_location_id == 1 && this.state.curent_location_id == -1){
      return (
        <View style={styles.container}>
          <Text style={styles.instructions}>
            Starting the tour
          </Text>
          <View>
            <Button
             onPress={this.onSubmitEdit}
              title="Lets go!!"
              color="#841584"
            />
          </View>
        </View>
      );
    }
    else if(this.state.curent_location_id == -1 && this.state.next_location_id != 1){
      return (
        <View style={styles.container}>
          <Text style={styles.instructions}>
            Lets go to the next place          
          </Text>
		  <ScrollView>
			{
				this.state.direction.split(';').map( item => {
					return (<Text> {item} </Text>);
				})
			}
		  </ScrollView>

          <View>
            <Button
             onPress={this.onSubmitEdit}
              title="Reached"
              color="#841584"
            />
          </View>
        </View>
      );
	}
	else if(this.state.curent_location_id == -1 && this.state.next_location_id != 1){
	  return (
	    <View style={styles.container}>
	      <Text style={styles.instructions}>
	        Lets go to the next place          
	      </Text>
		  <ScrollView>
		  {
		    this.state.direction.split(';').map( item => {
		      return (<Text> {item} </Text>);
		    })
		  }
		  </ScrollView>
	      <View>
	        <Button
	         onPress={this.onSubmitEdit}
	          title="Reached"
	          color="#841584"
	        />
	      </View>
	    </View>
	  	);
	}
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