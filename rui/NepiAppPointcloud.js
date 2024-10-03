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
import { Columns, Column } from "./Columns"
import Label from "./Label"
import Select, { Option } from "./Select"
import Styles from "./Styles"
import Button, { ButtonMenu } from "./Button"
import BooleanIndicator from "./BooleanIndicator"
import CameraViewer from "./CameraViewer"
import Input from "./Input"

import { round, convertStrToStrList, createShortValuesFromNamespaces, createMenuListFromStrList,
  onDropdownSelectedSendStr, onUpdateSetStateValue, onEnterSendFloatValue, onEnterSetStateFloatValue,
  } from "./Utilities"

import NepiPointcloudProcessControls from "./NepiAppPointcloudProcess"
import NepiPointcloudRenderControls from "./NepiAppPointcloudRender"

import NepiIFSaveData from "./Nepi_IF_SaveData"

@inject("ros")
@observer

// Pointcloud Application page
class PointcloudApp extends Component {
  constructor(props) {
    super(props)

    this.state = {
      appName: "app_pointcloud",
      appNamespace: null,
      pointcloudTopicList: [],
      selectedPointclouds: [],
      viewableTopics: false,
     
      showTransforms: false,
      transforms_topic_list: [],
      transforms_list: [],
      selectedTransformInd: 0,
      selectedTransformPointcloud: "NONE",
      selectedTransformData: null,
      selectedTransformTX: 0,
      selectedTransformTY: 0,
      selectedTransformTZ: 0,
      selectedTransformRX: 0,
      selectedTransformRY: 0,
      selectedTransformRZ: 0,
      selectedTransformHO: 0,
      age_filter_s: null,
      primary_pointcloud_topic: null,
      pointcloud_img_topic: null,


      combineOption: null,
      combineOptionsList: [],

      rangeMaxMeters: null,
      rangeMinMeters: null,

      connected: false,

      listener: null



    }

    this.updateSelectionListener = this.updateSelectionListener.bind(this)
    this.selectionStatusListener = this.selectionStatusListener.bind(this)

    this.getAppNamespace = this.getAppNamespace.bind(this)
    this.getPointcloudOptions = this.getPointcloudOptions.bind(this)
    this.getSelectedPointclouds = this.getSelectedPointclouds.bind(this)
    this.onTogglePointcloudSelection = this.onTogglePointcloudSelection.bind(this)
    this.toggleViewableTopics = this.toggleViewableTopics.bind(this)

    this.updateTranformsList = this.updateTranformsList.bind(this)
    
    this.onClickToggleShowTransforms = this.onClickToggleShowTransforms.bind(this)
    
    this.sendTransformUpdateMessage = this.sendTransformUpdateMessage.bind(this)
    this.sendClearTransformUpdateMessage = this.sendClearTransformUpdateMessage.bind(this)

    this.setSelectedTransform = this.setSelectedTransform.bind(this)



  }

  getAppNamespace(){
    const { namespacePrefix, deviceId} = this.props.ros
    var appNamespace = null
    if (namespacePrefix !== null && deviceId !== null){
      appNamespace = "/" + namespacePrefix + "/" + deviceId + "/" + this.state.appName
    }
    return appNamespace
  }

  // Callback for handling ROS Status messages
  selectionStatusListener(message) {
    const pointcloudTopicsStr = message.selected_pointcloud_topics
    var pointcloudsStrList = convertStrToStrList(pointcloudTopicsStr)
    const combineOptionsStr = message.combine_options
    const combineOptions = convertStrToStrList(combineOptionsStr)
    const framesList = convertStrToStrList(message.available_3d_frames)
    const transformsTopicStr = message.transforms_topic_list
    const transformsStr = message.transforms_list
    this.updateTranformsList( transformsTopicStr,transformsStr)
    this.setState({
    selectedPointclouds:pointcloudsStrList,
    frame_3d: message.output_3d_frame,
    frames3dlist: framesList,
    age_filter_s: message.age_filter_s,
    combineOptionsList: combineOptions,
    combineOption: message.combine_option,
    rangeMinMeters: message.range_min_max_m.start_range,
    rangeMaxMeters: message.range_min_max_m.stop_range,
    primary_pointcloud_topic: message.primary_pointcloud_topic
    })
    this.setState({connected: true})
    var image_topic = null
    if (message.publishing_pointcloud_img === true){
      image_topic = this.state.appNamespace + "/pointcloud_image"
    }
    else {
      image_topic = ""
    }
    this.setState({pointcloud_img_topic: image_topic})
    this.render()
  }

  // Function for configuring and subscribing to Status
  updateSelectionListener() {
    const statusNamespace = this.getAppNamespace() + '/status'
    if (this.state.listener) {
      this.state.listener.unsubscribe()
    }
    var listener = this.props.ros.setupPointcloudSelectionStatusListener(
          statusNamespace,
          this.selectionStatusListener
        )
    this.setState({ listener: listener})
  }

  // Lifecycle method called when compnent updates.
  // Used to track changes in the topic
  componentDidUpdate(prevProps, prevState, snapshot) {
    const namespace = this.getAppNamespace()
    if (prevState.appNamespace !== namespace && namespace !== null) {
      if (namespace.indexOf('null') === -1) {
        this.setState({appNamespace: namespace})
        this.updateSelectionListener()
      } 
    }
  }

  // Lifecycle method called just before the component umounts.
  // Used to unsubscribe to Status message
  componentWillUnmount() {
    if (this.state.listener) {
      this.state.listener.unsubscribe()
    }
  }

  // Function for creating image topic options.
  getPointcloudOptions() {
    const { pointcloudTopics } = this.props.ros
    const thisPointcloudTopic = this.state.appNamespace + "/pointcloud"
    var items = []
    var pointcloudTopicShortnames = createShortValuesFromNamespaces(pointcloudTopics)
    for (var i = 0; i < pointcloudTopics.length; i++) {
      if (pointcloudTopics[i] !== thisPointcloudTopic){
        items.push(<Option value={pointcloudTopics[i]}>{pointcloudTopicShortnames[i]}</Option>)
      }
    }
    //items.push(<Option>{"ALL"}</Option>) 
    items.push(<Option>{"NONE"}</Option>) 
    return items
  }



  onTogglePointcloudSelection(event){
    const {pointcloudTopics, sendStringMsg} = this.props.ros
    const pointcloud = event.target.value
    const addNamespace = this.state.appNamespace + "/add_pointcloud"
    const removeNamespace = this.state.appNamespace + "/remove_pointcloud"
    const selectedList = this.state.selectedPointclouds
    if (this.state.appNamespace){
      if (pointcloud === "NONE"){
        for (let ind = 0; ind < selectedList.length; ind++) {
          sendStringMsg(removeNamespace,selectedList[ind])
        }
      }
      else if (pointcloud === "ALL"){
        var pointcloud2
        for (let ind = 0; ind < pointcloudTopics.length; ind++) {
          pointcloud2 = pointcloudTopics[ind]
          if ( pointcloud2 !== "NONE" || pointcloud2 !== "ALL"){
            if (pointcloud2.indexOf('pointcloud_app') === -1 && selectedList.includes(pointcloud2) === false) {
              sendStringMsg(addNamespace,pointcloud2)
            }
          }
        }
      }
      else if (selectedList.indexOf(pointcloud) !== -1){
        sendStringMsg(removeNamespace,pointcloud)
      }
      else {
        sendStringMsg(addNamespace,pointcloud)
      }
    }
  }

 

  updateTranformsList(transformsTopicMsg,transformsMsg) {
    var topicsList = convertStrToStrList(transformsTopicMsg)
    const transformsFlat = convertStrToStrList(transformsMsg)
    var transform = []
    var transformsList = []
    var tf_index = 0
    for (var i = 0; i < transformsFlat.length; i++) {
      transform.push(transformsFlat[i])
      tf_index += 1
      if (tf_index === 7){
        transformsList.push(transform)
        transform = []
        tf_index = 0
      }
    }
    this.setState({transforms_topic_list: topicsList,
                  transforms_list: transformsList
    })
  }


  updateSelectedTransformData(topicIndex){
    const name = this.state.transforms_topic_list[topicIndex]
    const data = this.state.transforms_list[topicIndex]
    this.setState({selectedTransformInd: topicIndex,
      selectedTransformPointcloud: name,
      selectedTransformData: data,})
  }


  toggleViewableTopics() {
    const set = !this.state.viewableTopics
    this.setState({viewableTopics: set})
  }


  getSelectedPointclouds(){
    const selectedPointclouds = this.state.selectedPointclouds
    return selectedPointclouds
  }


  sendTransformUpdateMessage(){
    const {sendFrame3DTransformUpdateMsg} = this.props.ros
    const namespace = this.state.appNamespace + "/update_transform"
    const transformNamespace = this.state.selectedTransformPointcloud
    const TX = parseFloat(this.state.selectedTransformTX)
    const TY = parseFloat(this.state.selectedTransformTY)
    const TZ = parseFloat(this.state.selectedTransformTZ)
    const RX = parseFloat(this.state.selectedTransformRX)
    const RY = parseFloat(this.state.selectedTransformRY)
    const RZ = parseFloat(this.state.selectedTransformRZ)
    const HO = parseFloat(this.state.selectedTransformHO)
    const transformList = [TX,TY,TZ,RX,RY,RZ,HO]
    sendFrame3DTransformUpdateMsg(namespace,transformNamespace,transformList)
  }

  sendClearTransformUpdateMessage(){
     this.setState({
      selectedTransformTX: 0,
      selectedTransformTY: 0,
      selectedTransformTZ: 0,
      selectedTransformRX: 0,
      selectedTransformRY: 0,
      selectedTransformRZ: 0,
      selectedTransformHO: 0,      
    })
    const {sendFrame3DTransformUpdateMsg} = this.props.ros
    const namespace = this.state.appNamespace + "/update_transform"
    const transformNamespace = this.state.selectedTransformPointcloud
    const transformList = [0,0,0,0,0,0,0]
    sendFrame3DTransformUpdateMsg(namespace,transformNamespace,transformList)
  }


  
  
  onClickToggleShowTransforms(){
    const currentVal = this.state.showTransforms 
    this.setState({showTransforms: !currentVal})
    this.render()
  }

  setSelectedTransform(event){
    const pointcloud = event.target.value
    const pointclouds = this.state.transforms_topic_list
    const transforms = this.state.transforms_list
    const tf_index = pointclouds.indexOf(pointcloud)
    if (tf_index !== -1){
      this.setState({
        selectedTransformPointcloud: pointcloud,
        selectedTransformInd: tf_index
      })
      const transform = transforms[tf_index]
      this.setState({
        selectedTransformTX: round(transform[0]),
        selectedTransformTY: round(transform[1]),
        selectedTransformTZ: round(transform[2]),
        selectedTransformRX: round(transform[3]),
        selectedTransformRY: round(transform[4]),
        selectedTransformRZ: round(transform[5]),
        selectedTransformHO: round(transform[6])
      })
      
    }
  }

  renderPointcloudSelection() {
    const { saveConfigTriggered, sendTriggerMsg  } = this.props.ros
    const {viewableTopics} = this.state
    const pointcloudSources = this.getPointcloudOptions()
    const selectedPointclouds = this.getSelectedPointclouds()
    const NoneOption = <Option>None</Option>
    const connected = this.state.connected
    return (
      <React.Fragment>

            <Section title={"Selection"}>



            <Columns>
              <Column>

                  <Label title="Add/Remove Pointclouds">
                    <div onClick={this.toggleViewableTopics} style={{backgroundColor: Styles.vars.colors.grey0}}>
                      <Select style={{width: "10px"}}/>
                    </div>
                    <div hidden={!viewableTopics}>
                    {pointcloudSources.map((pointcloud) =>
                    <div onClick={this.onTogglePointcloudSelection}
                      style={{
                        textAlign: "center",
                        padding: `${Styles.vars.spacing.xs}`,
                        color: Styles.vars.colors.black,
                        backgroundColor: (selectedPointclouds.includes(pointcloud.props.value))? Styles.vars.colors.blue : Styles.vars.colors.grey0,
                        cursor: "pointer",
                        }}>
                        <body pointcloud-topic ={pointcloud} style={{color: Styles.vars.colors.black}}>{pointcloud}</body>
                    </div>
                    )}
                    </div>
                  </Label>

             </Column>
             <Column>

             <div align={"left"} textAlign={"left"} hidden={connected === false}>
                  <Label title={"Primary Pointcloud"}>
                    <Select
                      id="primary_pointcloud"
                      onChange={(event) => onDropdownSelectedSendStr.bind(this)(event,"/set_primary_pointcloud")}
                      value={this.state.primary_pointcloud_topic}
                    >
                      {this.state.selectedPointclouds
                        ? createMenuListFromStrList(this.state.selectedPointclouds, true, [],[],[])
                        : NoneOption}
                    </Select>
                  </Label>
              </div>

              </Column>
            </Columns>

            <div align={"left"} textAlign={"left"} hidden={connected === false}>

            <Label title={""}></Label>

            <Columns>
              <Column>
                  <Label title={"Age Filter (s)"}>
                    <Input id="age_filter" 
                      value={this.state.age_filter_s} 
                      onChange={(event) => onUpdateSetStateValue.bind(this)(event,"age_filter_s")} 
                      onKeyDown= {(event) => onEnterSendFloatValue.bind(this)(event,this.state.appNamespace + "/set_age_filter")} />
                  </Label>

                  <Label title={""}></Label>
                  <Label title={""}></Label>
        
              </Column>
              <Column>

                  <Label title={"Combine Options"}>
                    <Select
                      id="combine_options"
                      onChange={(event) => onDropdownSelectedSendStr.bind(this)(event,"/set_combine_option")}
                      value={this.state.combineOption}
                    >
                      {this.state.combineOptionsList
                        ? createMenuListFromStrList(this.state.combineOptionsList, false, [],[],[])
                        : NoneOption}
                    </Select>
                  </Label>
                  
                  <Label title={""}></Label>
                  <Label title={""}></Label>

              </Column>
            </Columns>


            <Columns>
              <Column>

 {/*                                
                  <Label title="Show 3D Transforms">
                    <Toggle
                      checked={this.state.showTransforms===true}
                      onClick={this.onClickToggleShowTransforms}>
                    </Toggle>
                  </Label>
*/}


                  <Label title={"Select Data Transform"}>
                    <Select
                      id="select_transforms"
                      onChange={this.setSelectedTransform}
                      value={this.state.selectedTransformPointcloud}
                    >
                      {this.state.transforms_topic_list
                        ? createMenuListFromStrList(this.state.transforms_topic_list, true, [],["NONE"],[])
                        : NoneOption}
                    </Select>
                  </Label>




              </Column>
              <Column>


              </Column>
            </Columns>


            <div align={"left"} textAlign={"left"} hidden={this.state.selectedTransformPointcloud === "NONE"}>

          <Columns>
            <Column>


              <Label title={"X (m)"}>
                <Input
                  value={this.state.selectedTransformTX}
                  id="XTranslation"
                  onChange= {(event) => onUpdateSetStateValue.bind(this)(event,"selectedTransformTX")}
                  onKeyDown= {(event) => onEnterSetStateFloatValue.bind(this)(event,"selectedTransformTX")}
                  style={{ width: "80%" }}
                />
              </Label>

              <Label title={"Y (m)"}>
                <Input
                  value={this.state.selectedTransformTY}
                  id="YTranslation"
                  onChange= {(event) => onUpdateSetStateValue.bind(this)(event,"selectedTransformTY")}
                  onKeyDown= {(event) => onEnterSetStateFloatValue.bind(this)(event,"selectedTransformTY")}
                  style={{ width: "80%" }}
                />
              </Label>

              <Label title={"Z (m)"}>
                <Input
                  value={this.state.selectedTransformTZ}
                  id="ZTranslation"
                  onChange= {(event) => onUpdateSetStateValue.bind(this)(event,"selectedTransformTZ")}
                  onKeyDown= {(event) => onEnterSetStateFloatValue.bind(this)(event,"selectedTransformTZ")}
                  style={{ width: "80%" }}
                />
              </Label>


              <ButtonMenu>
                <Button onClick={() => this.sendClearTransformUpdateMessage()}>{"Clear Transform"}</Button>
              </ButtonMenu>


            </Column>
            <Column>

              <Label title={"Roll (deg)"}>
                <Input
                  value={this.state.selectedTransformRX}
                  id="XRotation"
                  onChange= {(event) => onUpdateSetStateValue.bind(this)(event,"selectedTransformRX")}
                  onKeyDown= {(event) => onEnterSetStateFloatValue.bind(this)(event,"selectedTransformRX")}
                  style={{ width: "80%" }}
                />
              </Label>

              <Label title={"Pitch (deg)"}>
                <Input
                  value={this.state.selectedTransformRY}
                  id="YRotation"
                  onChange= {(event) => onUpdateSetStateValue.bind(this)(event,"selectedTransformRY")}
                  onKeyDown= {(event) => onEnterSetStateFloatValue.bind(this)(event,"selectedTransformRY")}
                  style={{ width: "80%" }}
                />
              </Label>

              <Label title={"Yaw (deg)"}>
                <Input
                  value={this.state.selectedTransformRZ}
                  id="ZRotation"
                  onChange= {(event) => onUpdateSetStateValue.bind(this)(event,"selectedTransformRZ")}
                  onKeyDown= {(event) => onEnterSetStateFloatValue.bind(this)(event,"selectedTransformRZ")}
                  style={{ width: "80%" }}
                />
              </Label>

              <ButtonMenu>
                <Button onClick={() => this.sendTransformUpdateMessage()}>{"Update Transform"}</Button>
              </ButtonMenu>

            </Column>
          </Columns>
          </div>

    
          <Columns>
            <Column>
            

            </Column>
            <Column>
              <Label title={""}></Label>

              <ButtonMenu>
                <Button onClick={() => saveConfigTriggered(this.state.appNamespace)}>{"Save Config"}</Button>
              </ButtonMenu>

            </Column>
            <Column>
            
              <Label title={""}></Label>

              <ButtonMenu>
                <Button onClick={() => sendTriggerMsg(this.state.appNamespace + "/reset_app")}>{"Reset App"}</Button>
              </ButtonMenu>

            </Column>
          </Columns>


          </div>
          
        </Section>

        
      </React.Fragment>
    )
  }

  renderImageViewer() {
    const img_topic = this.state.pointcloud_img_topic
    return (
      <React.Fragment>
        <Columns>
          <Column equalWidth={false}>
            <CameraViewer
              imageTopic={img_topic}
              title={"pointcloud_image"}
              hideQualitySelector={false}
            />
          </Column>
        </Columns>
      </React.Fragment>
    )
  }


  render() {
    const connected = this.state.connected
    return (



      <Columns>
        <Column>

        <Label title={"App Running"}>
           <BooleanIndicator value={(this.state.connected)} />
        </Label>

          {this.renderImageViewer()}

          <div hidden={!connected}>
            <NepiIFSaveData
                  saveNamespace={this.state.appNamespace}
                  title={"Nepi_IF_SaveData"}
              />
          </div>


        </Column>
        <Column>

          {this.renderPointcloudSelection()}




        <div hidden={!connected}>

        <NepiPointcloudProcessControls
                processNamespace={this.state.appNamespace + "/process"}
                title={"NepiPointcloudProcessControls"}
            />
            
          <NepiPointcloudRenderControls
                renderNamespace={this.state.appNamespace + "/render"}
                title={"NepiPointcloudRenderControls"}
            />

        </div>


         </Column>
      </Columns>

    )
  }
}

export default PointcloudApp
