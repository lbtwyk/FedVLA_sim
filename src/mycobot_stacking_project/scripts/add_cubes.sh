#!/bin/bash

# Wait for Gazebo to start
sleep 5

# Add the yellow cube
gz service -s /world/cube_stacking_world/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 1000 \
  --req 'sdf: '\
'<?xml version="1.0" ?>'\
'<sdf version="1.7">'\
'<model name="yellow_cube">'\
'<pose>0.22 0.12 0.04 0 0 0</pose>'\
'<static>false</static>'\
'<link name="link">'\
'<inertial>'\
'<mass>0.1</mass>'\
'<inertia>'\
'<ixx>2.6666e-5</ixx>'\
'<ixy>0</ixy>'\
'<ixz>0</ixz>'\
'<iyy>2.6666e-5</iyy>'\
'<iyz>0</iyz>'\
'<izz>2.6666e-5</izz>'\
'</inertia>'\
'</inertial>'\
'<collision name="collision">'\
'<geometry>'\
'<box>'\
'<size>0.04 0.04 0.04</size>'\
'</box>'\
'</geometry>'\
'</collision>'\
'<visual name="visual">'\
'<geometry>'\
'<box>'\
'<size>0.04 0.04 0.04</size>'\
'</box>'\
'</geometry>'\
'<material>'\
'<ambient>1 1 0 1</ambient>'\
'<diffuse>1 1 0 1</diffuse>'\
'<specular>0.1 0.1 0.1 1</specular>'\
'</material>'\
'</visual>'\
'</link>'\
'</model>'\
'</sdf>'

# Add the orange cube
gz service -s /world/cube_stacking_world/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 1000 \
  --req 'sdf: '\
'<?xml version="1.0" ?>'\
'<sdf version="1.7">'\
'<model name="orange_cube">'\
'<pose>0.4 0.15 0.04 0 0 0</pose>'\
'<static>false</static>'\
'<link name="link">'\
'<inertial>'\
'<mass>0.1</mass>'\
'<inertia>'\
'<ixx>2.6666e-5</ixx>'\
'<ixy>0</ixy>'\
'<ixz>0</ixz>'\
'<iyy>2.6666e-5</iyy>'\
'<iyz>0</iyz>'\
'<izz>2.6666e-5</izz>'\
'</inertia>'\
'</inertial>'\
'<collision name="collision">'\
'<geometry>'\
'<box>'\
'<size>0.04 0.04 0.04</size>'\
'</box>'\
'</geometry>'\
'</collision>'\
'<visual name="visual">'\
'<geometry>'\
'<box>'\
'<size>0.04 0.04 0.04</size>'\
'</box>'\
'</geometry>'\
'<material>'\
'<ambient>1 0.65 0 1</ambient>'\
'<diffuse>1 0.65 0 1</diffuse>'\
'<specular>0.1 0.1 0.1 1</specular>'\
'</material>'\
'</visual>'\
'</link>'\
'</model>'\
'</sdf>'

echo "Cubes added to the simulation"
