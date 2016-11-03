QUESTIONS:
1- What is the max range for the laser?


GAZEBO WHAT TO DO:

http://people.oregonstate.edu/~chungje/Code/Pioneer3dx%20simulation/ros-indigo-gazebo2-pioneer.pdf
	(if with problems launching gazebo -> http://answers.gazebosim.org/question/4153/gazebo-crashes-immediately-using-roslaunch-after-installing-gazebo-ros-packages/)

Modifications to the tutorial:

1- Modified run_pioneer_gazebo to launch---> roslaunch pioneer_gazebo pioneer_world.launch &
pid="$pid $!" 

2- Modified pioneer_world.launch to use our world file ---->  <arg name="world_name" value="$(find pioneer_gazebo)/worlds/FifthFloorWorld.world"/>


3- Modified pioneer.gazebo to have the correct measurements for the laser -----> 

			  <!-- hokuyo -->
			  <gazebo reference="laser">
			    <sensor type="ray" name="head_hokuyo_sensor">
			      <pose>0 0 0 0 0 0</pose>
			      <visualize>true</visualize>
			      <update_rate>40</update_rate>
			      <ray>
			        <scan>
			          <horizontal>
			            <samples>720</samples>
			            <resolution>1</resolution>
			            <min_angle>-2.09</min_angle>
			            <max_angle>2.09</max_angle>
			          </horizontal>
			        </scan>
			        <range>
			          <min>0.2</min>
			          <max>5.6</max>
			          <resolution>0.01</resolution>
			        </range>
			        <noise>
			          <type>gaussian</type>
			          <!-- Noise parameters based on published spec for Hokuyo laser
			               achieving "+-30mm" accuracy at range < 10m.  A mean of 0.0m and
			               stddev of 0.01m will put 99.7% of samples within 0.03m of the true
			               reading. -->
			          <mean>0.0</mean>
			          <stddev>0.01</stddev>
			        </noise>
			      </ray>
			      <plugin name="gazebo_ros_head_hokuyo_controller" filename="libgazebo_ros_laser.so">
			        <topicName>base_scan</topicName>
			        <frameName>laser</frameName>
			      </plugin>
			    </sensor>

			  </gazebo>


4- Created a custom Rviz view and changed pioneer_rviz.launch file --->
  <!-- Show in Rviz   -->
  <node name="rviz" pkg="rviz" type="rviz" args="-d $(find pioneer_description)/launch/CustomOnlyMapConfig.rviz"/>
