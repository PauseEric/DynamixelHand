import mujoco
import mujoco.viewer

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




# def controller(model, data):
#     # Access current joint positions or other data
#     current_qpos = data.qpos 

#     # Calculate control torques or target positions/velocities
#     # e.g., simple Proportional (P) control for a single joint:
#     target_angle = 1.57 # radians
#     error = target_angle - current_qpos[0]
#     data.ctrl[0] = error * Kp # Kp is a proportional gain


def main():
    print("Running Main Sequence")
     #Launch the interactive viewer
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Keep the viewer running until the user closes it
        while viewer.is_running():
            data.ctrl[prox_pointer_actuator_id]=0
            data.ctrl[mid_thumb_actuator_id]=0
            data.ctrl[distal_thumb_actuator_id]=0.5
            data.ctrl[prox_thumb_actuator_id]=0.5
            # Step the simulation
            mujoco.mj_step(model, data)
            #Sim Additional Updates Here
            

            # Update the viewer
            viewer.sync()


if __name__ == "__main__":
    main()