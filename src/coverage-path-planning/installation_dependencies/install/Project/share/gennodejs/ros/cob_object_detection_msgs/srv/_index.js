
"use strict";

let StartObjectRecording = require('./StartObjectRecording.js')
let BagTrainObject = require('./BagTrainObject.js')
let DetectObjects = require('./DetectObjects.js')
let AcquireObjectImage = require('./AcquireObjectImage.js')
let SaveRecordedObject = require('./SaveRecordedObject.js')
let BaTestEnvironment = require('./BaTestEnvironment.js')
let StopObjectRecording = require('./StopObjectRecording.js')
let TrainObject = require('./TrainObject.js')

module.exports = {
  StartObjectRecording: StartObjectRecording,
  BagTrainObject: BagTrainObject,
  DetectObjects: DetectObjects,
  AcquireObjectImage: AcquireObjectImage,
  SaveRecordedObject: SaveRecordedObject,
  BaTestEnvironment: BaTestEnvironment,
  StopObjectRecording: StopObjectRecording,
  TrainObject: TrainObject,
};
