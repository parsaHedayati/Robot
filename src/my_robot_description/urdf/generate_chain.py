from ikpy import chain
from ikpy.urdf.utils import get_urdf_tree



spider_f1_links = ['base_link','torso_joint_f_1','femur_1','foot-f-1','torso_joint_f_2','femur_2','foot-f-2'
                   ,'torso_joint_b_1','femur_3','foot-b-1','torso_joint_b_2','femur_4','foot-b-2']
spider_f1_joints = ['base_torso_f_1','torso_f_1_femur_1','foot-f-1_femur-1','base_torso_f_2','torso_f_2_femur_2','foot-f-2_femur-2'
                    ,'base_torso_b_1','torso_b_1_femur_3','foot-b-1_femur-3','base_torso_b_2','torso_b_2_femur_4','foot-b-2_femur-4']

spider_elements = [
    'base_link', 'base_torso_f_1', 
    'torso_joint_f_1', 'torso_f_1_femur_1', 
    'femur_1', 'foot-f-1_femur-1', 
    'foot-f-1', 'base_torso_f_2', 
    'torso_joint_f_2', 'torso_f_2_femur_2', 
    'femur_2', 'foot-f-2_femur-2', 
    'foot-f-2', 'base_torso_b_1', 
    'torso_joint_b_1', 'torso_b_1_femur_3', 
    'femur_3', 'foot-b-1_femur-3', 
    'foot-b-1', 'base_torso_b_2', 
    'torso_joint_b_2', 'torso_b_2_femur_4', 
    'femur_4', 'foot-b-2_femur-4', 
    'foot-b-2'
]


# Define the kinematic chain
spider_chain = chain.Chain.from_urdf_file(
    "urdf/my_robot.urdf.xacro",
    base_elements=spider_elements,
    last_link_vector=[0, 0, 0.1],  # Adjust based on your end-effector
    active_links_mask=
        3*[False]+4*[True]+7*[False],  
    
    symbolic= False,
    name="spider_json"
)

# Save the chain configuration to a JSON file
spider_chain.to_json_file(force=True)



