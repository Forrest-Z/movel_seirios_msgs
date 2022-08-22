# Aux Task Manager
https://docs.google.com/document/d/1W8DJe40wU3rdGmeF_ROb2Af_slhKbVS1iKbn97uU3xs/edit
Manages starting, cancelling and monitoring auxiliary tasks with ros topics.

## How it works
- aux_task_manager_node subscribes to topic /aux_task_manager/request published by web.
- Then it processes the request. After which it reports the status by publishing the topic /aux_task_manager/status.
- While alive, aux_task_manager continuously monitors running tasks and publish status to update.

## Topics
- Subscribes to `/aux_task_manager/request` 
- Publishes to `/aux_task_manager/status`
