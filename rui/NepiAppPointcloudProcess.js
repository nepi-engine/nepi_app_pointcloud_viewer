/*
 * Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
 *
 * This file is part of nepi-engine
 * (see https://github.com/nepi-engine).
 *
 * License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
 */
import React, { Component } from "react"
import { observer, inject } from "mobx-react"

import Section from "./Section"
//import EnableAdjustment from "./EnableAdjustment"
import Button, { ButtonMenu } from "./Button"
import Toggle from "react-toggle"
import Label from "./Label"
import { Column, Columns } from "./Columns"
import Input from "./Input"
import Styles from "./Styles"
import Select, { Option } from "./Select"


import { onUpdateSetStateValue, onEnterSendFloatValue, onEnterSendIntValue, onChangeSwitchStateValue, createMenuListFromStrList, onDropdownSelectedSendStr } from "./Utilities"

@inject("ros")
@observer

// Component that contains the  Pointcloud App Viewer Controls
class NepiPointcloudProcessControls extends Component {
  constructor(props) {
    super(props)

    // these states track the values through  Status messages
    this.state = {
      show_process_controls: false,
      range_clip_enabled: false,
      clip_target_topic: null,
      range_clip_min_m: null,
      range_clip_max_m: null,
      voxel_downsample_size_m: null,
      uniform_downsample_points: null,
      outlier_k_points: null,
      framesList: ['nepi_center_frame','sensor_frame','map'],
      frame_3d: null,
      clip_options: [],
      clip_selection: null,

      processListener: null,

    }

    this.onEnterSendInputBoxRangeWindowValue = this.onEnterSendInputBoxRangeWindowValue.bind(this)
  
    this.updateProcessListener = this.updateProcessListener.bind(this)
    this.processStatusListener = this.processStatusListener.bind(this)

  }

  // Callback for handling ROS Status messages
  processStatusListener(message) {
    
    this.setState({
      range_clip_enabled: message.clip_enabled,
      range_clip_min_m: message.clip_meters.start_range,
      range_clip_max_m: message.clip_meters.stop_range,
      clip_target_topic: message.clip_target_topic,
      voxel_downsample_size_m: message.voxel_downsample_size_m,
      uniform_downsample_points: message.uniform_downsample_points,
      outlier_k_points: message.outlier_k_points,
      clip_options: message.clip_options,
      clip_selection: message.clip_selection

    })
  }

  // Function for configuring and subscribing to Status
  updateProcessListener() {
    const statusNamespace = this.props.processNamespace + '/status'
    if (this.state.processListener) {
      this.state.processListener.unsubscribe()
    }
    var processListener = this.props.ros.setupPointcloudProcessStatusListener(
          statusNamespace,
          this.processStatusListener
        )
    this.setState({ processListener: processListener})
  }

  // Lifecycle method called when compnent updates.
  // Used to track changes in the topic
  componentDidUpdate(prevProps, prevState, snapshot) {
    const { processNamespace } = this.props
    if (prevProps.processNamespace !== processNamespace && processNamespace !== null) {
      if (processNamespace.indexOf('null') === -1){
        this.updateProcessListener()
        this.render()
      } 
    }
  }

  // Lifecycle method called just before the component umounts.
  // Used to unsubscribe to Status message
  componentWillUnmount() {
    if (this.state.processListener) {
      this.state.processListener.unsubscribe()
    }
  }



  onEnterSendInputBoxRangeWindowValue(event, topicName, entryName) {
    const {publishRangeWindow} = this.props.ros
    const namespace = this.props.processNamespace + topicName
    if(event.key === 'Enter'){
      const value = parseFloat(event.target.value)
      if (!isNaN(value)){
        var min = this.state.range_clip_min_m
        var max = this.state.range_clip_max_m
        if (entryName === "min"){
          min = value
        }
        else if (entryName === "max"){
          max = value
        }
        publishRangeWindow(namespace,min,max,false)
      }
      document.getElementById(event.target.id).style.color = Styles.vars.colors.black
    }
  }

  render() {
    const NoneOption = <Option>None</Option>
    const {  sendTriggerMsg, sendBoolMsg } = this.props.ros
    return (
      <Section title={"Process Controls"}>


      <Columns>
      <Column>
          <Label title="Show Process Controls">
                <Toggle
                checked={this.state.show_process_controls===true}
                onClick={() => onChangeSwitchStateValue.bind(this)("show_process_controls",this.state.show_process_controls)}>
                </Toggle>
          </Label>
      </Column>
      <Column>

      <div hidden={!this.state.show_process_controls}>
          <ButtonMenu>
                    <Button onClick={() => sendTriggerMsg( this.props.processNamespace + "/reset_controls")}>{"Reset Controls"}</Button>
              </ButtonMenu>
      </div>
      </Column>
      </Columns>

      <div hidden={!this.state.show_process_controls}>

          <Columns>
          <Column>
            <Label title="Clip Range Enabled">
                  <Toggle
                  checked={this.state.range_clip_enabled===true}
                  onClick={() => sendBoolMsg(this.props.processNamespace + "/set_clip_enable",!this.state.range_clip_enabled)}>
                  </Toggle>
            </Label>
          

            <div hidden={!this.state.range_clip_enabled}>
            <div hidden={!this.state.show_process_controls}>
            <Label title={"Set Clip Ranges (m)"}>
            </Label>
            </div>
            </div>


           </Column>
           <Column>
           <Label title={"Clip Selection"}>
                    <Select
                      id="clip_selection"
                        onChange={(event) => onDropdownSelectedSendStr.bind(this)(event, this.props.processNamespace + "/set_clip_selection")}
                        value={this.state.clip_selection}
                      >
                        {this.state.clip_options
                          ? createMenuListFromStrList(this.state.clip_options, false, [],[],[])
                          : NoneOption}
                      </Select>
                    </Label>


           </Column>
          </Columns>
          <div hidden={!this.state.range_clip_enabled}>
          <div hidden={!this.state.show_process_controls}>
          <Columns>
          <Column>

          <Label title={"Set Range Clip Min"}>
                    <Input id="set_range_clip_min" 
                      value={this.state.range_clip_min_m} 
                      onChange={(event) => onUpdateSetStateValue.bind(this)(event,"range_clip_min_m")} 
                      onKeyDown= {(event) => this.onEnterSendInputBoxRangeWindowValue(event,"/set_range_clip_m","min")} />
              </Label>
            
              </Column>
              <Column>
                  <Label title={"Set Range Clip Max"}>
                    <Input id="set_range_clip_max" 
                     value={this.state.range_clip_max_m} 
                      onChange={(event) => onUpdateSetStateValue.bind(this)(event,"range_clip_max_m")} 
                      onKeyDown= {(event) => this.onEnterSendInputBoxRangeWindowValue(event,"/set_range_clip_m","max")} />                      
                  </Label>  

           </Column>
          </Columns>  
          </div>
          </div>

          <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>    


          <Columns>
            <Column>

              <Label title={"Pointclud Filtering"}></Label>
                      
{/*              <Label title={"Uniform Downsample k Points"}>
                <Input id="uniform_downsample_k_points" 
                  value={this.state.uniform_downsample_points} 
                  onChange={(event) => onUpdateSetStateValue.bind(this)(event,"uniform_downsample_points")} 
                  onKeyDown= {(event) => this.onEnterSendInputBoxIntValue(event,"/uniform_downsample_k_points")} />
              </Label>
*/}

              <Label title={"Outlier Removal k Points"}>
                <Input id="outlier_k_points" 
                  value={this.state.outlier_k_points} 
                  onChange={(event) => onUpdateSetStateValue.bind(this)(event,"outlier_k_points")} 
                  onKeyDown= {(event) => onEnterSendIntValue.bind(this)(event,this.props.processNamespace + "/outlier_removal_num_neighbors")} />
              </Label>

              <Label title={"Voxel Downsample Size (m)"}>
                <Input id="voxel_downsample_size_m" 
                  value={this.state.voxel_downsample_size_m} 
                  onChange={(event) => onUpdateSetStateValue.bind(this)(event,"voxel_downsample_size_m")} 
                  onKeyDown= {(event) => onEnterSendFloatValue.bind(this)(event,this.props.processNamespace + "/set_voxel_downsample_size")} />
              </Label>

            </Column>
            <Column>

            </Column>
          </Columns>  

        </div>
        
      </Section>
    )
  }

}
export default NepiPointcloudProcessControls
