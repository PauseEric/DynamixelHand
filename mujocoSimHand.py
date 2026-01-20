import mujoco
import mujoco.viewer
import math
import time
# Define the path to your URDF file
urdf_path = "testurdf/urdf/testurdf.urdf"
xml_path = "dex_for_urdf/urdf/mjmodel.xml"
# Load the model from the XML path
model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

prox_thumb_actuator_id = model.actuator('pos_proxthumb').id
prox_pointer_actuator_id = model.actuator('pos_proxpointer').id
prox_middle_actuator_id = model.actuator('pos_proxmiddle').id
prox_ring_actuator_id = model.actuator('pos_proxring').id   
prox_pinky_actuator_id = model.actuator('pos_proxpinky').id

mid_thumb_actuator_id = model.actuator('pos_midthumb').id
mid_pointer_actuator_id = model.actuator('pos_midpointer').id
mid_middle_actuator_id = model.actuator('pos_midmiddle').id
mid_ring_actuator_id = model.actuator('pos_midring').id
mid_pinky_actuator_id = model.actuator('pos_midpinky').id

distal_thumb_actuator_id = model.actuator('pos_distalthumb').id
distal_pointer_actuator_id = model.actuator('pos_distalpointer').id
distal_middle_actuator_id = model.actuator('pos_distalmiddle').id
distal_ring_actuator_id = model.actuator('pos_distalring').id
distal_pinky_actuator_id = model.actuator('pos_distalpinky').id

#List to keep track of actuator positions
pointer_tri_pos = [0,0,0] #Format is [proximal_pos, middle_pos, distal_pos]
middle_tri_pos = [0,0,0]
ring_tri_pos = [0,0,0]
pinky_tri_pos = [0,0,0]

#Calculating the Joint positions for non real motors
#Stage 1 of Finger links distance lengths (all in inches)

proximal_finger_length = 1.45 #a1
proximal_connector_length = 0.3 #b1
proximal_tendon_length = 1.45 #c1
proximal_anchor_distance = 0.35 #d1 //inches
starting_angle_offset = 37.54 #degree offset used for initial pose calculation (approximated)


mid_finger_length = 1.05 #a1
mid_connector_length = 0.35 #b1
mid_tendon_length = 1 #c1
mid_anchor_distance = 0.25 #d1 //inches
mid_joint_theta = 0.0
global mid_joint_offset, distal_joint_offset

op_proximal_theta = 0.0
op_mid_theta = 0.0
op_distal_theta = 0.0
distal_joint_theta = 0.0

#mid_link_length = 0.3 #inches

# def controller(model, data):
#     # Access current joint positions or other data
#     current_qpos = data.qpos 

#     # Calculate control torques or target positions/velocities
#     # e.g., simple Proportional (P) control for a single joint:
#     target_angle = 1.57 # radians
#     error = target_angle - current_qpos[0]
#     data.ctrl[0] = error * Kp # Kp is a proportional gain


def calculatePos(current_joint_pos):
    # Convert current joint position from radians to degrees
    theta2 = (math.pi - (current_joint_pos) - math.radians(starting_angle_offset))
    # print(theta2)
    # Freudenstein's equation components (in radians, convert to degrees at the end) Proximal Calc
    k1= proximal_anchor_distance/proximal_finger_length
    k2= proximal_anchor_distance/proximal_tendon_length
    k3= (proximal_finger_length**2 - proximal_connector_length**2 + proximal_tendon_length**2 + proximal_anchor_distance**2)/(2*proximal_finger_length*proximal_tendon_length)
    A= math.sin(theta2)
    B= k2+ math.cos(theta2)
    C= k1 * math.cos(theta2) + k3 
    # print(A)
    # print(B)
    # print(C)
    theta4= 2* math.atan(( A + math.sqrt(A**2 +B**2 - C**2))/ (C+B)) # A+- math.sqrt() possible as it is a quadratic
    op_proximal_theta = math.pi - theta4 - (math.pi-theta2)
   
    prox_cross_length= (proximal_anchor_distance*(math.sin(theta2)))/math.sin(op_proximal_theta)
    op_mid_theta = math.asin(((proximal_finger_length-prox_cross_length)*math.sin(op_proximal_theta))/proximal_connector_length)

    # Freudenstein's equation components (in radians, convert to degrees at the end) Middle Calc
    g1= mid_anchor_distance/mid_finger_length
    g2= mid_anchor_distance/mid_tendon_length
    g3= (mid_finger_length**2 - mid_connector_length**2 + mid_tendon_length**2 + mid_anchor_distance**2)/(2*mid_finger_length*mid_tendon_length)
    Aa= math.sin(op_mid_theta)
    Bb= g2+ math.cos(op_mid_theta)
    Cc= g1 * math.cos(op_mid_theta) + g3
    theta5= 2* math.atan(( Aa - math.sqrt(Aa**2 +Bb**2 - Cc**2))/ (Cc+Bb)) # Aa+- math.sqrt() possible as it is a quadratic

    mid_joint_theta= math.pi - theta5 #for mid joint
    op_distal_theta = math.pi - mid_joint_theta- op_mid_theta
    
    mid_cross_length= (mid_anchor_distance*(math.sin(op_mid_theta)))/math.sin(op_distal_theta)
    distal_joint_theta= math.asin(((mid_finger_length-mid_cross_length)*math.sin(op_distal_theta))/mid_connector_length) #for distal joint 
    
    mid_joint_theta= mid_joint_theta - mid_joint_offset
    distal_joint_theta= distal_joint_theta -distal_joint_offset

    joint_tri_value = [current_joint_pos, mid_joint_theta, distal_joint_theta]
    return (joint_tri_value)

###Individual Link Control  --- For testing
def thumbController(prox, mid, dist):
    data.ctrl[prox_thumb_actuator_id] = prox
    data.ctrl[mid_thumb_actuator_id] = mid
    data.ctrl[distal_thumb_actuator_id] = dist
def pointerController(prox, mid, dist):
    data.ctrl[prox_pointer_actuator_id] = prox
    data.ctrl[mid_pointer_actuator_id] = mid
    data.ctrl[distal_pointer_actuator_id] = dist
def middleController(prox, mid, dist):
    data.ctrl[prox_middle_actuator_id] = prox
    data.ctrl[mid_middle_actuator_id] = mid 
    data.ctrl[distal_middle_actuator_id] = dist
def ringController(prox, mid, dist):
    data.ctrl[prox_ring_actuator_id] = prox
    data.ctrl[mid_ring_actuator_id] = mid
    data.ctrl[distal_ring_actuator_id] = dist
def pinkyController(prox, mid, dist):
    data.ctrl[prox_pinky_actuator_id] = prox
    data.ctrl[mid_pinky_actuator_id] = mid
    data.ctrl[distal_pinky_actuator_id] = dist

###Group Finger Control --- Use with the calculatePos function
#eg: pointerController(calculatePos(data.sensor('sensor_proxthumb').data[0]));

def pointerGroup(joint_list):
    data.ctrl[prox_pointer_actuator_id] = joint_list[0]
    data.ctrl[mid_pointer_actuator_id] = joint_list[1]
    data.ctrl[distal_pointer_actuator_id] = joint_list[2]
def middleGroup(joint_list):
    data.ctrl[prox_middle_actuator_id] = joint_list[0]
    data.ctrl[mid_middle_actuator_id] = joint_list[1]
    data.ctrl[distal_middle_actuator_id] = joint_list[2]
def ringGroup(joint_list):
    data.ctrl[prox_ring_actuator_id] = joint_list[0]
    data.ctrl[mid_ring_actuator_id] = joint_list[1]
    data.ctrl[distal_ring_actuator_id] = joint_list[2]
def pinkyGroup(joint_list):
    data.ctrl[prox_pinky_actuator_id] = joint_list[0]
    data.ctrl[mid_pinky_actuator_id] = joint_list[1]
    data.ctrl[distal_pinky_actuator_id] = joint_list[2]


def setOffsets():
    print("setup")
    thumbController(0,0,0)
    pointerController(0,0,0)
    middleController(0,0,0)
    ringController(0,0,0)
    pinkyController(0,0,0)
    time.sleep(1)
    offset_list = calculatePos(0)
    mid_joint_offset= offset_list[1]
    distal_joint_offset= offset_list[2]
    # print(offset_list)
    # print(mid_joint_offset)
    # print(distal_joint_offset) 
    print(calculatePos(0))
    print(calculatePos(1))
    print(calculatePos(1.5))
 
    
def main():
    print("Running Main Sequence")
    setOffsets()
     #Launch the interactive viewer
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Keep the viewer running until the user closes it
        while viewer.is_running():
            
            pointerGroup(calculatePos(1))
            middleGroup(calculatePos(1.25))
            
           # print(data.sensor('sensor_proxthumb').data[0])

            # Step the simulation
            mujoco.mj_step(model, data)
            #Sim Additional Updates Here

        
            # Update the viewer
            viewer.sync()


if __name__ == "__main__":
    main()