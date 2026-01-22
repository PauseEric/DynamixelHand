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

prox_thumb_actuator_id = model.actuator('motor_proxthumb').id
prox_pointer_actuator_id = model.actuator('motor_proxpointer').id
prox_middle_actuator_id = model.actuator('motor_proxmiddle').id
prox_ring_actuator_id = model.actuator('motor_proxring').id   
prox_pinky_actuator_id = model.actuator('motor_proxpinky').id

mid_thumb_actuator_id = model.actuator('motor_midthumb').id

distal_thumb_actuator_id = model.actuator('motor_distalthumb').id



        
def main():
    print("Running Main Sequence")
    print("press p to check current motor positions")
     #Launch the interactive viewer
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Keep the viewer running until the user closes it
        while viewer.is_running():
            if (data.sensor('sensor_proxpointer').data[0] <= (math.pi/2)):
                data.ctrl[prox_pointer_actuator_id] = 1
            else:
                data.ctrl[prox_pointer_actuator_id] = 0
            print(data.sensor('sensor_proxpointer').data[0])

            # pointerGroup(calcPos(1))
            # middleGroup(calcPos(0.5))
            # ringGroup(calcPos(1))
            # pinkyGroup(calcPos(1.5))
          #  if (data.sensor('sensor_proxpointer').data[0] != pointerTarget):
            # print(data.sensor('sensor_proxpointer').data[0])
            # print(data.sensor('sensor_midpointer').data[0])
            # print(data.sensor('sensor_distalpointer').data[0])
           # print(data.sensor('sensor_proxthumb').data[0])

            # Step the simulation
            mujoco.mj_step(model, data)
            #Sim Additional Updates Here

        
            # Update the viewer
            viewer.sync()


if __name__ == "__main__":
    main()