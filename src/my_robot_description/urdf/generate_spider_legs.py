from ikpy import chain
from ikpy.urdf.utils import get_urdf_tree



spider_f1_links = ['base_link','torso_joint_f_1','femur_1','foot-f-1']
spider_f1_joints = ['base_torso_f_1','torso_f_1_femur_1','foot-f-1_femur-1']

spider_f1_elements = ['base_link'] + ['base_torso_f_1']+['torso_joint_f_1']+ ['torso_f_1_femur_1']+['femur_1']+['foot-f-1_femur-1']+ ['foot-f-1']


spider_f1_chain = chain.Chain.from_urdf_file(
    "urdf/my_robot.urdf.xacro",
    base_elements=spider_f1_elements,
    last_link_vector=[0, 0, 0],  # Modify if needed based on your end-effector
    active_links_mask=[False] +  2*[True] + [False] + [True],  # 12 joints, all active except the base
    symbolic=False,
    name="spider_f1"
)

# Save the chain configuration to a JSON file
spider_f1_chain.to_json_file(force=True)


spider_f2_links = ['base_link','torso_joint_f_2','femur_2','foot-f-2']
spider_f2_joints = ['base_torso_f_2','torso_f_2_femur_2','foot-f-2_femur-2']


spider_f2_elements = [x for pair in zip(spider_f2_links, spider_f2_joints) for x in pair]


spider_f2_chain = chain.Chain.from_urdf_file(
    "urdf/my_robot.urdf.xacro",
    base_elements=spider_f2_elements,
    last_link_vector=[0, 0, 0],  # Modify if needed based on your end-effector
    active_links_mask=[False] + 2*[True] + [False] + [True],  # 12 joints, all active except the base
    symbolic=False,
    name="spider_f2"
)

spider_f2_chain.to_json_file(force=True)
# Optional: print to verify
print(spider_f2_chain)



spider_b1_links = ['base_link','torso_joint_b_1','femur_3','foot-b-1']
spider_b1_joints = ['base_torso_b_1','torso_b_1_femur_3','foot-b-1_femur-3']

spider_b1_elements = [x for pair in zip(spider_b1_links, spider_b1_joints) for x in pair]


spider_b1_chain = chain.Chain.from_urdf_file(
    "urdf/my_robot.urdf.xacro",
    base_elements=spider_b1_elements,
    last_link_vector=[0, 0, 0],  # Modify if needed based on your end-effector
    active_links_mask=[False] +  2*[True] + [False] + [True],  # 12 joints, all active except the base
    symbolic=False,
    name="spider_b1"
)

# Save the chain configuration to a JSON file
spider_b1_chain.to_json_file(force=True)


spider_b2_links = ['base_link','torso_joint_b_2','femur_4','foot-b-2']
spider_b2_joints = ['base_torso_b_2','torso_b_2_femur_4','foot-b-2_femur-4']

spider_b2_elements = [x for pair in zip(spider_b2_links, spider_b2_joints) for x in pair]


spider_b2_chain = chain.Chain.from_urdf_file(
    "urdf/my_robot.urdf.xacro",
    base_elements=spider_b2_elements,
    last_link_vector=[0, 0, 0],  # Modify if needed based on your end-effector
    active_links_mask=[False] +  2*[True] + [False] + [True],  # 12 joints, all active except the base
    symbolic=False,
    name="spider_b2"
)

# Save the chain configuration to a JSON file
spider_b2_chain.to_json_file(force=True)
