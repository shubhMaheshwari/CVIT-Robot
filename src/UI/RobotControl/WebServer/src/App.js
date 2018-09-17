import React, { Component } from 'react';
import MultiToggle from 'react-multi-toggle';
import './App.css';

import EventEmitter2 from 'eventemitter2'

window.EventEmitter2 = EventEmitter2.EventEmitter2
var RosClient = require("roslibjs-client");
var speed = 15;
var client = new RosClient({
  url: "ws://10.2.132.18:9090"
});
client.on("connected", function() {
  console.log("Connection established!");
});
client.on("disconnected", function() {
  console.log("Connection disconnected!");
});

class VideoFeed extends Component {
  constructor(props) {
    super(props);
    this.state = {
      uri:'data:image/jpeg;base64,/9j/4AAQSkZJRgABAQEAYABgAAD/2wBDAAIBAQIBAQICAgICAgICAwUDAwMDAwYEBAMFBwYHBwcGBwcICQsJCAgKCAcHCg0KCgsMDAwMBwkODw0MDgsMDAz/2wBDAQICAgMDAwYDAwYMCAcIDAwMDAwMDAwMDAwMDAwMDAwMDAwMDAwMDAwMDAwMDAwMDAwMDAwMDAwMDAwMDAwMDAz/wAARCAABAAEDASIAAhEBAxEB/8QAHwAAAQUBAQEBAQEAAAAAAAAAAAECAwQFBgcICQoL/8QAtRAAAgEDAwIEAwUFBAQAAAF9AQIDAAQRBRIhMUEGE1FhByJxFDKBkaEII0KxwRVS0fAkM2JyggkKFhcYGRolJicoKSo0NTY3ODk6Q0RFRkdISUpTVFVWV1hZWmNkZWZnaGlqc3R1dnd4eXqDhIWGh4iJipKTlJWWl5iZmqKjpKWmp6ipqrKztLW2t7i5usLDxMXGx8jJytLT1NXW19jZ2uHi4+Tl5ufo6erx8vP09fb3+Pn6/8QAHwEAAwEBAQEBAQEBAQAAAAAAAAECAwQFBgcICQoL/8QAtREAAgECBAQDBAcFBAQAAQJ3AAECAxEEBSExBhJBUQdhcRMiMoEIFEKRobHBCSMzUvAVYnLRChYkNOEl8RcYGRomJygpKjU2Nzg5OkNERUZHSElKU1RVVldYWVpjZGVmZ2hpanN0dXZ3eHl6goOEhYaHiImKkpOUlZaXmJmaoqOkpaanqKmqsrO0tba3uLm6wsPExcbHyMnK0tPU1dbX2Nna4uPk5ebn6Onq8vP09fb3+Pn6/9oADAMBAAIRAxEAPwD9/KKKKAP/2Q==',
    };
  }
  componentDidMount() {
    client.topic.subscribe('/webcam_image', 'sensor_msgs/CompressedImage', (message) => {
      this.setState({
        uri:"data:image/jpeg;base64,"+message.data,
        prevURI1: this.state.uri,
      });
    });
  }
  render() {
    return (
      <img src={this.state.uri} style={{width:"100%"}}/>
    );
  }
}

class JoyStick extends Component {  
  componentDidMount() {
    var options = {
      zone: document.getElementById('zone_joystick'),
      color: 'red',
      mode: 'semi',
      size: 150,
      catchDistance: '75',
      position: {left: '50%', top: '50%'},
    };
    var manager = require('nipplejs').create(options);
    manager.on('added', function (evt, nipple) {
        nipple.on('move', function (evt, data) {
          if(data.angle.degree >= 45 && data.angle.degree < 135) {
            client.topic.publish('/movement', 'std_msgs/String', {'data':JSON.stringify({mode: "MANUAL",wheel1: speed, wheel2: speed})});
          }
          else if(data.angle.degree >= 135 && data.angle.degree < 225) {
            client.topic.publish('/movement', 'std_msgs/String', {'data':JSON.stringify({mode: "MANUAL",wheel1: -(speed), wheel2: (speed)})});
          }
          else if(data.angle.degree >= 225 && data.angle.degree < 320) {
            client.topic.publish('/movement', 'std_msgs/String', {'data':JSON.stringify({mode: "MANUAL",wheel1: -speed, wheel2: -speed})});
          }
          else{
            client.topic.publish('/movement', 'std_msgs/String', {'data':JSON.stringify({mode: "MANUAL",wheel1: (speed), wheel2: -(speed)})});
          }
        }).on('end', function(evt, data) {
          client.topic.publish('/movement', 'std_msgs/String', {'data':JSON.stringify({mode: "MANUAL",wheel1: 0, wheel2: 0})});
        });
    })
  }
  componentWillUnmount() {
    document.getElementById('zone_joystick').innerHTML = '';
  }
  render() {
    return (
      <div id="zone_joystick" style={{height:"200px", position: 'relative'}}></div>
    );
  }
}

class App extends Component {

  constructor(props) {
    super(props);
    this.state = {
      controlMode: 'MANUAL',
      speedMode: 15,
      speakerValue: '',
    };
  }

  handleModeChange = (value) => {
    client.topic.publish('/robot_mode', 'std_msgs/String', {'data': value})
    this.setState({controlMode: value});
  }


  handleSpeedChange = (value) => {
    speed = value;
    this.setState({speedMode: value});
  }

  handleSpeakChange = (event) => {
    this.setState({speakerValue: event.target.value});
  }

  speakFunction = (event) => {
    client.topic.publish('/speaker', 'std_msgs/String', {'data': this.state.speakerValue})
  }

  modeOptions = [
    {
      displayName: 'Tracking',
      value: 'TRACK'
    },
    {
      displayName: 'Random',
      value: 'RANDOM'
    },
    {
      displayName: 'Manual',
      value: 'MANUAL'
    },
  ];

  speedOptions = [
    {
      displayName: 'Normal',
      value: 15
    },
    {
      displayName: 'Boost',
      value: 25
    },
  ];

  render() {
    return (
      <div>
        <VideoFeed/>
        <div style={{width: '100%', display: 'flex', }}>
          <input className='speakText' style={{flexGrow: '1'}} type="text" value={this.state.speakerValue} onChange={this.handleSpeakChange} placeholder="I'll speak the text here"/>
          <input className='speakButton' type='button' value='Speak' onClick={this.speakFunction} />
        </div>
        <div>
          <div style={{width: '300px'}}>
            <MultiToggle
              options={this.modeOptions}
              selectedOption={this.state.controlMode}
              onSelectOption={this.handleModeChange}
              label="Select Control Mode"
            />
          </div>
          <div style={{width: '200px'}}>
            <MultiToggle
              options={this.speedOptions}
              selectedOption={this.state.speedMode}
              onSelectOption={this.handleSpeedChange}
              label="Select Speed"
            />
          </div>
        </div>
        <JoyStick/>
      </div>
    );
  }
}

export default App;
