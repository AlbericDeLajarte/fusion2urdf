import xml.etree.ElementTree as ET
import os, sys
import subprocess

import struct
import shutil
import shlex

def parse_stl(stl_file):
    with open(stl_file, mode='rb') as file: 
        fileContent = file.read()
    n_faces = struct.unpack('i', fileContent[80:84])[0]
    assert (len(fileContent)-84)/50 == n_faces

    x = []
    y = []
    z = []
    for i in range(n_faces):
        face = struct.unpack('f'*12+'H', fileContent[84+50*i:84+50*(i+1)])
        x.append(min(face[3], face[6], face[9]))
        y.append(min(face[4], face[7], face[10]))
        z.append(min(face[5], face[8], face[11]))

    return min(x), max(x), min(y), max(y), min(z), max(z)


class PostProcess:
    def __init__(self, xacro_file):
        self.xacro_file = xacro_file
        self.urdf_path = xacro_file.replace('.xacro', '.urdf')
        self.output_path = os.path.join(os.sep, *xacro_file.split(os.sep)[:-2])

    '''
    Convert xacro to urdf
    '''
    def xacro_to_urdf(self):
        file_path = os.path.dirname(os.path.abspath(__file__))

        python_path = sys.executable.split( '/' )
        python_path = python_path[ :python_path.index( 'Contents' ) + 1 ]
        python_path = '/'.join( python_path ) + '/' + 'Frameworks/Python.framework/Versions/Current/bin/python'
        python_path = shlex.quote(python_path)

        shutil.copy(f'{file_path}/xacro.py', self.output_path)
        result = subprocess.run(f'cd {self.output_path} && {python_path} xacro.py -o {self.urdf_path} {self.xacro_file}', capture_output=True, text=True, shell=True)
        
        return result
        
    '''
    Post process the URDF file to remove unnecessary tags and replace mesh with simple shapes
    '''
    def post_process(self):
        tree = ET.parse(self.urdf_path)
        root = tree.getroot()

        package_name = self.output_path.split(os.sep)[-1]

        toRemove = []
        for link in root:
            
            # Not necessary for simulation
            if link.tag in ['transmission', 'gazebo']: 
                toRemove.append(link)

            if link.tag == 'link':
                for geometry_type in ['collision']:

                    # Get nodes
                    collision_xml = link.find(geometry_type)
                    geometry = collision_xml.find('geometry')
                    mesh_xml = geometry.find('mesh')

                    # get mesh properties
                    mesh_file = self.output_path + mesh_xml.attrib['filename'].split(package_name)[-1]
                    scale = mesh_xml.attrib['scale']
                    scale = [float(s) for s in scale.split()]

                    # Fit box to mesh
                    min_x, max_x, min_y, max_y, min_z, max_z = parse_stl(mesh_file)

                    dx = abs(max_x - min_x) * scale[0]
                    dy = abs(max_y - min_y) * scale[1]
                    dz = abs(max_z - min_z) * scale[2]

                    # Position is relative to parent joint
                    # STL file coordinate are already in world frame, so original compensate for that
                    position = [float(point) for point in collision_xml.find('origin').attrib['xyz'].split()]
                    position = [    (max_x + min_x)*scale[0]/2 + position[0], 
                                    (max_y + min_y)*scale[0]/2 + position[1], 
                                    (max_z + min_z)*scale[0]/2 + position[2] ]
                    

                    # Replace mesh with simple shape
                    shape_xml = ET.Element('box', {'size': f'{dx} {dy} {dz}'})
                    geometry.append(shape_xml)

                    collision_xml.find('origin').attrib['xyz'] = f'{position[0]} {position[1]} {position[2]}'
                    geometry.remove(mesh_xml)

        for link in toRemove:
            root.remove(link)

        tree.write(self.urdf_path.replace('.urdf', '_optimised.urdf'))



if __name__ == '__main__':

    file_path = os.path.dirname(os.path.abspath(__file__))
    xacro_file = os.path.join(file_path, '..', '..', 'Example', 'Basic_Robot_description', 'urdf', 'Basic_Robot.xacro')
    
    post_process = PostProcess(xacro_file)
    post_process.xacro_to_urdf()
    post_process.post_process()