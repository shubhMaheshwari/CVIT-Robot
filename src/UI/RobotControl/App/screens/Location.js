import React from 'react';
import { StyleSheet, View, Picker, Button} from 'react-native';
import {Text, WebView} from 'react-native';
import {Component} from 'react';
import {IndicatorViewPager, PagerTitleIndicator} from 'rn-viewpager';

import EventEmitter2 from 'eventemitter2'
window.EventEmitter2 = EventEmitter2.EventEmitter2
var RosClient = require("roslibjs-client");
var client = new RosClient({
  url: "ws://10.2.132.18:9090"
});

var data = require('../static/data.json');
 
export default class ViewPagerPage extends Component {
  constructor(props) {
    super(props);
    this.myRef = React.createRef();
  }
  onPageChanged = (event) => {
    if(event.position == 4) {
      this.myRef.current.setPage(0);
    }
  }
  alertPickUp = (event) => {
    client.topic.publish('/speaker', 'std_msgs/String', {'data': 'Please pick the message from the bot.'});
  }
  render() {
    return (
      <View style={{flex:1}}>
        <IndicatorViewPager
          style={{flex:1, backgroundColor:'white'}}
          indicator={this._renderTitleIndicator()}
          onPageSelected={this.onPageChanged}
          ref={this.myRef}
        >
          <View style={{padding: 10, backgroundColor:'#db3e5e'}}>
            <Text style={{fontSize: 20}}>Navigate the bot to your location</Text>
            <WebView
              source={{uri: 'http://10.2.132.18:3000'}}
              injectedJavaScript={`const meta = document.createElement('meta'); meta.setAttribute('content', 'width=device-width, initial-scale=0.9, maximum-scale=0.9, user-scalable=0'); meta.setAttribute('name', 'viewport'); document.getElementsByTagName('head')[0].appendChild(meta); `}
              scalesPageToFit={false}
            />
          </View>
          <View style={{padding: 10, backgroundColor:'cornflowerblue'}}>
            <Text style={{fontSize: 20}}>Place the message on the bot and then go to the next step</Text>
          </View>
          <View style={{padding: 10, backgroundColor:'#1AA094'}}>
            <Text style={{fontSize: 20}}>Navigate the bot to the specified location</Text>
            <WebView
              source={{uri: 'http://10.2.132.18:3000'}}
              injectedJavaScript={`const meta = document.createElement('meta'); meta.setAttribute('content', 'width=device-width, initial-scale=0.9, maximum-scale=0.9, user-scalable=0'); meta.setAttribute('name', 'viewport'); document.getElementsByTagName('head')[0].appendChild(meta); `}
              scalesPageToFit={false}
            />
          </View>
          <View style={{padding: 10, backgroundColor:'#FFDE03'}}>
            <Text style={{fontSize: 20, marginBottom: 20}}>Press the button to alert the person to pick the message up</Text>
            <Button
              onPress={this.alertPickUp}
              title="Alert"
              color="#8e53c0"
              accessibilityLabel="Alert the person to pick the object up"
            />
          </View>
          <View style={{padding: 10, backgroundColor:'#db3e5e'}}>
          </View>
        </IndicatorViewPager>
      </View>
    );
  }
 
  _renderTitleIndicator() {
    return <PagerTitleIndicator titles={['Step 1', 'Step 2', 'Step 3', 'Step 4', 'Restart']} />;
  }
 
}