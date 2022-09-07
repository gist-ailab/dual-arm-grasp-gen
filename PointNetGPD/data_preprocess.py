import os
import shutil
import csv
import argparse
import subprocess

def read_metadata_info(metadata):
    file = open(metadata, 'r')
    reader = csv.DictReader(file)
    
    return reader


def convert_to_watertight(input_mesh, output_mesh):
    print(f'watertight running on "{input_mesh}"')
    #!TODO: This custom path should be modified after sudo make install
    cmd = f'/home/kangmin/catkin_ws/src/Manifold/build/manifold {input_mesh} {output_mesh}'
    retobj = subprocess.run(cmd, capture_output=True, shell=True, text=True)
    if retobj.returncode != 0:
        print(f'manifold failed on "f{input_mesh}"')
        if retobj.stdout != '': print(f'{retobj.stdout}')
        if retobj.stderr != '': print(f'{retobj.stderr}')
        

def convert_obj_to_sdf(obj, dim, padding):
    print(f'converting running on "{obj}"')
    #!TODO: This custom path should be modified after sudo make install
    cmd = '/home/kangmin/catkin_ws/src/SDFGen/bin/SDFGen \"%s\" %d %d' % (obj, dim, padding)
    retobj = subprocess.run(cmd, capture_output=True, shell=True, text=True)
    if retobj.returncode != 0:
        print(f'SDFGen failed on "f{obj}"')
        

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--datatrans", dest="datatrans", action="store_true")
    args = parser.parse_args()
    
    origin_data_path = 'data/shapenetcore/03001627' #* chair category
    origin_trans_path = 'data/shapenetcore/rexchair' #* rex chair sub category
    metadata_path = 'data/shapenetcore/03001627.csv'
    
    #*needed parameter for convert obj to sdf 
    dim = 100
    padding = 5
    
    if args.datatrans:
        
        if not os.path.exists(origin_trans_path):
            os.mkdir(origin_trans_path)

        info = read_metadata_info(metadata_path)
        for row in info:
            if 'rex chair' in row['wnlemmas']:
                _, filename = row['fullId'].split('.')
                file_path = os.path.join(origin_data_path, filename)
                trans_path = os.path.join(origin_trans_path, filename)
                shutil.copytree(file_path, trans_path, dirs_exist_ok=True)
    
    #* change obj to watertight and to sdf file
    file_list = os.listdir(origin_trans_path)
    for i in file_list:
        input_mesh = os.path.join(origin_trans_path, i, 'model.obj')
        output_mesh = os.path.join(origin_trans_path, i, 'watertight_model.obj')
        convert_to_watertight(input_mesh, output_mesh)
        convert_obj_to_sdf(output_mesh, dim, padding)
    
    
                
if __name__=='__main__':
    main()