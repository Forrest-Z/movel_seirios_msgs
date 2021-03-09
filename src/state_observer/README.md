# Movel State Diag

## 1. Overview

This package provide a diagnostic tool that can monitor the running states of nodes and other information within the nodes for troubleshooting. It is built on the basis of [the diagnostic package](http://wiki.ros.org/diagnostics).

Nodes of interest can be configured to publish [diagnostic_msgs/DiagnosticArray](http://docs.ros.org/api/diagnostic_msgs/html/msg/DiagnosticArray.html) messages with the device names, status and data points on the topic /Diagnostics.

Diagnostic aggregator will subscribe to /Diagnostics, process the message and publish on the topic /Diagnostics_agg. The aggregator holds a list of nodes that are desired to monitor. Any node in the list that is not running will be marked "Stale" in the aggregator since it is not publishing messages on /Diagnostics.

The state manager takes use of this feature to monitor whether a node is currently running or not.

## 2. Quick Launch

```
roslaunch movel_state_diag state_manager_diag.launch
```
Node states are published on the topic /robot_state and can be viewed by tospicd echo.

```
rostopic echo robot_state
```
## 3. Subscribed Topics

*diagnostics_agg* (diagnostic_msgs/DiagnosticArray)
* Diagnostic messages published by the diagnostic aggregator.

## 4. Published Topics

*robot_state* (movel_seirios_msgs/DiagnosticStateArray)
* The state information of the nodes.

## 5. Configuration

### 5.1 Adding diagnostic tasks to nodes

Tools in [diagnostic_updater](http://wiki.ros.org/diagnostic_updater?distro=melodic) assist in integrating diagnostics to a node. 
A working example [diagnostic_updater/src/example.cpp](https://docs.ros.org/api/diagnostic_updater/html/example_8cpp_source.html) shows the common uses of the updater. Below are the necessary steps needed.

``` 
 #include <diagnostic_updater/diagnostic_updater.h>
 #include <diagnostic_updater/publisher.h>
```
Include the header file to use the updater.

```
 diagnostic_updater::Updater updater;
```
Create an updater object. The updater class will publish [diagnostic_msgs/DiagnosticArray](http://docs.ros.org/api/diagnostic_msgs/html/msg/DiagnosticArray.html) messages to /diagnostics. The publication rate can be altered by changing the "~diagnostic_period" ros parameter.

```
updater.update();
```
Publish the message on the /diagnostics topic. We can call the method whenever is convenient. It will take care of the publishing rate as definded by "~diagnostic_period".

```
updater.add("Diagnostic task", diagnostic_task_func);

```

Adding diagnostic task to the updater with a name and a task function. It will be run when the updater decides to update.


```
void diagnostic_task_func(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
	//formatted text
	stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "The value of data is %f", value);

	//unformatted text
	stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Everything seems to be ok.");

	//unformatted text
  stat.add("Key Name", "Value String");
  stat.add("Key Name", value_double); //automatically handles conversion to string
	//formatted text
  stat.addf("Key Name", "The values are %f and %f", value_1, value_2);
}

```

DiagnosticStatusWrapper is a derived class of diagnostic_msgs::DiagnosticStatus. It provides methods that are convinient to use when filling out DiagnosticStatus messages. The methods summary and summaryf are used to set the status level and message, while add and addf are used to append key_value pairs.

```
DummyClass dc;
updater.add("Diagnostic task", &dc, &DummyClass::diagnostic_task_method);

```
Adding diagnostic task to the updater with a name and a task method.

```
class DummyClass
{
  public:
  void diagnostic_task_method(diagnostic_updater::DiagnosticStatusWrapper &stat)
 {
	stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Everything seems to be ok.");
  stat.add("Key Name", "Value");
 }

};

```
Adding diagnostic task using a method inside a class.

### 5.2 Configuring the diagnostic aggregator
The [diagnostic aggregator](http://wiki.ros.org/diagnostic_aggregator?distro=melodic) is given parameters that defines the nodes we want to monitor. These parameters are in the private namespace of the aggregator node.

```
pub_rate: 1.0 # Optional
base_path: '' # Optional, prepended to all diagnostic output
analyzers:
  motor: #namespace
    type: diagnostic_aggregator/GenericAnalyzer
    timeout: 5.0
    contains: ['motor_node']
  sensor: #namespace
    type: diagnostic_aggregator/GenericAnalyzer
    timeout: 5.0
    contains: ['sensor_node']
```

*type*: Mandatory. It tells the aggregator_node to load the plugin class GenericAnalyzer.
*timeout*: The node will appear as "Stale" if it has not receive any message for timeout priod.
*contains*: The node name.

### 5.3 Launching the diagnostic aggregator
Add the constructed yaml file to a roslaunch file so it loads in the aggregator_node's namespace.
The launch file should look like:

```
<launch>
  <node pkg="diagnostic_aggregator" type="aggregator_node"
        name="diagnostic_aggregator" >
    <!-- Load the yaml file above -->
    <rosparam command="load" 
              file="$(find my_package)/diagnostics.yaml" />
  </node>
</launch>

```
