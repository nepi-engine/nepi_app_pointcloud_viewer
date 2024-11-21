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
import RangeAdjustment from "./RangeAdjustment"
import { SliderAdjustment} from "./AdjustmentWidgets"
import Toggle from "react-toggle"
import Label from "./Label"
import { Column, Columns } from "./Columns"
import Input from "./Input"
import Styles from "./Styles"


@inject("ros")
@observer

// Component that contains the  Pointcloud App Viewer Controls
class NepiPointcloudRenderControls extends Component {
  constructor(props) {
    super(props)

    // these states track the values through  Status messages
    this.state = {
      renderEnabled: false,
      standardImageSizeStrList: null,
      rangeRatioMax: null,
      rangeRatioMin: null,
      rangeLimitMinM: null,
      rangeLimitMaxM: null,
      zoomAdjustment: null,
      rotateAdjustment: null,
      tiltAdjustment: null,
      camViewX: null,
      camViewY: null,
      camViewZ: null,
      camPosX: null,
      camPosY: null,
      camPosZ: null,
      camRotX: null,
      camRotY: null,
      camRotZ: null,

      renderListener: null,

    }

    this.onKeyCamText = this.onKeyCamText.bind(this)

    this.updateRenderListener = this.updateRenderListener.bind(this)
    this.renderStatusListener = this.renderStatusListener.bind(this)

    this.onUpdateSetStateValue = this.onUpdateSetStateValue.bind(this)
    
    
  }

  // Callback for handling ROS Status messages
  renderStatusListener(message) {
    this.setState({
      renderEnabled: message.render_enable,
      standardImageSizeStrList: message.standard_image_sizes,
      rangeRatioMin: message.range_clip_ratios.start_range,
      rangeRatioMax: message.range_clip_ratios.stop_range,
      rangeLimitMinM: message.range_min_max_m.start_range,
      rangeLimitMaxM: message.range_min_max_m.stop_range,
      zoomAdjustment: message.zoom_ratio,
      rotateAdjustment: message.rotate_ratio,
      tiltAdjustment: message.tilt_ratio,
      camViewX: message.camera_view.x,
      camViewY: message.camera_view.y,
      camViewZ: message.camera_view.z,
      camPosX: message.camera_position.x,
      camPosY: message.camera_position.y,
      camPosZ: message.camera_position.z,
      camRotX: message.camera_rotation.x,
      camRotY: message.camera_rotation.y,
      camRotZ: message.camera_rotation.z,
    })
    const frames3d = (this.state.frames3d)
    var frames3dlist = ["map"]
    for (let ind = 0; ind < frames3d.length; ind++) {
      frames3dlist.push(frames3d[ind])
    }
    this.setState({frame3dList: frames3dlist})
  }

  // Function for configuring and subscribing to Status
  updateRenderListener() {
    const statusNamespace = this.props.renderNamespace + '/status'
    if (this.state.renderListener) {
      this.state.renderListener.unsubscribe()
    }
    var renderListener = this.props.ros.setupStatusListener(
      statusNamespace,
      "nepi_app_pointcloud_viewer/PointcloudRenderStatus",
      this.renderStatusListener
    )
    this.setState({ renderListener: renderListener })
  }

  // Lifecycle method called when compnent updates.
  // Used to track changes in the topic
  componentDidUpdate(prevProps, prevState, snapshot) {
    const { renderNamespace } = this.props
    if (prevProps.renderNamespace !== renderNamespace && renderNamespace !== null) {
      if (renderNamespace.indexOf('null') === -1){
        this.updateRenderListener()
      } 
    }
  }


  // Lifecycle method called just before the component umounts.
  // Used to unsubscribe to Status message
  componentWillUnmount() {
    if (this.state.renderListener) {
      this.state.renderListener.unsubscribe()
    }
  }


  onKeyCamText(e,topic, f1, f2, f3) {
    const {sendFloatVector3Msg} = this.props.ros
    var namespace = topic
    var f1s = String(f1)
    var f2s = String(f2)
    var f3s = String(f3)
    if(e.key === 'Enter'){
      sendFloatVector3Msg(namespace,f1s,f2s,f3s)
      document.getElementById(e.target.id).style.color = Styles.vars.colors.black
    }
  }

  onUpdateSetStateValue(event,stateVarStr) {
    var key = stateVarStr
    var value = event.target.value
    var obj  = {}
    obj[key] = value
    this.setState(obj)
    document.getElementById(event.target.id).style.color = Styles.vars.colors.red
    this.render()
  }

  render() {
    const namespace = this.props.renderNamespace
    const {  sendTriggerMsg } = this.props.ros

    return (
      <Section title={"Render Controls"}>

   
        <Columns>
          <Column>
 
          <Label title="Render Enabled">
              <Toggle
              checked={this.state.renderEnabled===true}
              onClick={() => this.props.ros.sendBoolMsg(namespace + "/set_render_enable",!this.state.renderEnabled)}>
              </Toggle>
            </Label>
 

          </Column>
          <Column>
          </Column>
        </Columns>
        <div >
        <div hidden={!this.state.renderEnabled}>    
          <Columns>
            <Column>

            </Column>
            <Column>
            <div align={"left"} textAlign={"left"} >
                <ButtonMenu>
                  <Button onClick={() => sendTriggerMsg( namespace + "/reset_controls")}>{"Reset Controls"}</Button>
                </ButtonMenu>
              </div>
            </Column>
          </Columns>
        


            <div >
              <RangeAdjustment
                title="Range Clip"
                min={this.state.rangeRatioMin}
                max={this.state.rangeRatioMax}
                min_limit_m={this.state.rangeLimitMinM}
                max_limit_m={this.state.rangeLimitMaxM}
                topic={namespace + "/set_range_ratios"}
                tooltip={"Adjustable range"}
                unit={"m"}
              />
            </div>


            <div >

                <SliderAdjustment
                      title={"Zoom"}
                      msgType={"std_msgs/Float32"}
                      adjustment={this.state.zoomAdjustment}
                      topic={namespace + "/set_zoom_ratio"}
                      scaled={0.01}
                      min={0}
                      max={100}
                      tooltip={"Zoom controls for pointcloud image rendering"}
                      unit={"%"}
                  />


                <SliderAdjustment
                      title={"Rotate"}
                      msgType={"std_msgs/Float32"}
                      adjustment={this.state.rotateAdjustment}
                      topic={namespace + "/set_rotate_ratio"}
                      scaled={0.01}
                      min={0}
                      max={100}
                      tooltip={"Rotate controls for pointcloud image rendering"}
                      unit={"%"}
                  />

                  <SliderAdjustment
                      title={"Tilt"}
                      msgType={"std_msgs/Float32"}
                      adjustment={this.state.tiltAdjustment}
                      topic={namespace + "/set_tilt_ratio"}
                      scaled={0.01}
                      min={0}
                      max={100}
                      tooltip={"Tilt controls for pointcloud image rendering"}
                      unit={"%"}
                  />

              <Label title={"Camera View"}>
              </Label>
              <Columns>
                <Column>

                  <Label title={"X  "}>
                    <Input
                      id="camViewX"
                      value={(this.state.camViewX)}
                      onChange={(event) => this.onUpdateSetStateValue(event,"camViewX")}
                      onKeyDown={(event) => this.onKeyCamText(event,(namespace + "/set_camera_view"),this.state.camViewX,this.state.camViewY,this.state.camViewZ)}
                    />
                  </Label>

                </Column>
                <Column>

                <Label title={"Y  "}>
                    <Input
                      id="camViewY"
                      value={(this.state.camViewY)}
                      onChange={(event) => this.onUpdateSetStateValue(event,"camViewY")}
                      onKeyDown={(event) => this.onKeyCamText(event,(namespace + "/set_camera_view"),this.state.camViewX,this.state.camViewY,this.state.camViewZ)}
                    />
                  </Label>

                </Column>
                <Column>

                <Label title={"Z  "}>
                    <Input
                      id="camViewZ"
                      value={(this.state.camViewZ)}
                      onChange={(event) => this.onUpdateSetStateValue(event,"camViewZ")}
                      onKeyDown={(event) => this.onKeyCamText(event,(namespace + "/set_camera_view"),this.state.camViewX,this.state.camViewY,this.state.camViewZ)}
                    />
                  </Label>

                </Column>
              </Columns>


              <Label title={"Camera Position"}>
              </Label>
              <Columns>
                <Column>

                  <Label title={"X  "}>
                    <Input
                      id="camPosX"
                      value={(this.state.camPosX)}
                      onChange={(event) => this.onUpdateSetStateValue(event,"camPosX")}
                      onKeyDown={(event) => this.onKeyCamText(event,(namespace + "/set_camera_position"),this.state.camPosX,this.state.camPosY,this.state.camPosZ)}
                    />
                  </Label>

                </Column>
                <Column>

                <Label title={"Y  "}>
                    <Input
                      id="camPosY"
                      value={(this.state.camPosY)}
                      onChange={(event) => this.onUpdateSetStateValue(event,"camPosY")}
                      onKeyDown={(event) => this.onKeyCamText(event,(namespace + "/set_camera_position"),this.state.camPosX,this.state.camPosY,this.state.camPosZ)}
                    />
                  </Label>

                </Column>
                <Column>

                <Label title={"Z  "}>
                    <Input
                      id="camPosZ"
                      value={(this.state.camPosZ)}
                      onChange={(event) => this.onUpdateSetStateValue(event,"camPosZ")}
                      onKeyDown={(event) => this.onKeyCamText(event,(namespace + "/set_camera_position"),this.state.camPosX,this.state.camPosY,this.state.camPosZ)}
                    />
                  </Label>

                </Column>
              </Columns>


              <Label title={"Camera Rotation"}>
              </Label>
              <Columns>
                <Column>

                  <Label title={"X  "}>
                    <Input
                      id="camRotX"
                      value={(this.state.camRotX)}
                      onChange={(event) => this.onUpdateSetStateValue(event,"camRotX")}
                      onKeyDown={(event) => this.onKeyCamText(event,(namespace + "/set_camera_rotation"),this.state.camRotX,this.state.camRotY,this.state.camRotZ)}
                    />
                  </Label>

                </Column>
                <Column>

                <Label title={"Y  "}>
                    <Input
                      id="camRotY"
                      value={(this.state.camRotY)}
                      onChange={(event) => this.onUpdateSetStateValue(event,"camRotY")}
                      onKeyDown={(event) => this.onKeyCamText(event,(namespace + "/set_camera_rotation"),this.state.camRotX,this.state.camRotY,this.state.camRotZ)}
                    />
                  </Label>

                </Column>
                <Column>

                <Label title={"Z  "}>
                    <Input
                      id="camRotZ"
                      value={(this.state.camRotZ)}
                      onChange={(event) => this.onUpdateSetStateValue(event,"camRotZ")}
                      onKeyDown={(event) => this.onKeyCamText(event,(namespace + "/set_camera_rotation"),this.state.camRotX,this.state.camRotY,this.state.camRotZ)}
                    />
                  </Label>

                </Column>
              </Columns>
              </div>
            </div>
            
          </div>
        
      </Section>
    )
  }

}
export default NepiPointcloudRenderControls
