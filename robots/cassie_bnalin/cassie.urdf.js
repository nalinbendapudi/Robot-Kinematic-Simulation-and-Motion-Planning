/*
     Cassie robot URDF kinematic description in JavaScript
     
     @author M. Eva Mungai 

     based on https://github.com/UMich-BipedLab/Cassie_Model/blob/master/urdf/cassie.urdf

*/


robot = {
  name:"cassie", 
  base:"pelvis", 
  origin:{ xyz: [0,1.2,0], rpy:[0,0,0] },
  links: {
    "pelvis": {
      visual : { 
        origin : { xyz: [0,0,0], rpy:[0,0,0] },
        geometry : { mesh : { filename : "meshes/pelvis.stl" } },
        material : { color : { rgba : [0.35, 0.35, 0.35, 1] } }		
      }
    },
    "left_pelvis_abduction": {
      visual : { 
        origin : { xyz: [0,0,0], rpy:[0,0,0] },
        geometry : { mesh : { filename : "meshes/abduction.stl" } },
        material : { color : { rgba : [0.75, 0.75, 0.75, 1] } }
      }
    },	
    "left_pelvis_rotation": {
      visual : { 
        origin : { xyz: [0,0,0], rpy:[0,0,0] },
        geometry : { mesh : { filename : "meshes/yaw.stl" } },
        material : { color : { rgba : [0.75, 0.75, 0.75, 1] } }
      }
    },		
    "left_hip": {
      visual : { 
        origin : { xyz: [0,0,0], rpy:[0,0,0] },
        geometry : { mesh : { filename : "meshes/hip.stl" } },
        material : { color : { rgba : [0.75, 0.75, 0.75, 1] } }
      }
    },			
    "left_thigh": {
      visual : { 
        origin : { xyz: [0,0,0], rpy:[0,0,0] },
        geometry : { mesh : { filename : "meshes/thigh.stl" } },
        material : { color : { rgba : [0.75, 0.75, 0.75, 1] } }
      }
    },				
    "left_knee": {
      visual : { 
        origin : { xyz: [0,0,0], rpy:[0,0,0] },
        geometry : { mesh : { filename : "meshes/knee-output.stl" } },
        material : { color : { rgba : [0.35, 0.35, 0.35, 1] } }
      }
    },			
    "left_shin": {
      visual : { 
        origin : { xyz: [0,0,0], rpy:[0,0,0] },
        geometry : { mesh : { filename : "meshes/shin-bone.stl" } },
        material : { color : { rgba : [0.35, 0.35, 0.35, 1] } }
      }
    },		
    "left_tarsus": {
      visual : { 
        origin : { xyz: [0,0,0], rpy:[0,0,0] },
        geometry : { mesh : { filename : "meshes/tarsus.stl" } },
        material : { color : { rgba : [0.35, 0.35, 0.35, 1] } }
      }
    },			
    "left_toe": {
      visual : { 
        origin : { xyz: [0,0,0], rpy:[0,0,0] },
        geometry : { mesh : { filename : "meshes/toe.stl" } },
        material : { color : { rgba : [0.75, 0.75, 0.75, 1] } }
      }
    },		
    "right_pelvis_abduction": {
      visual : { 
        origin : { xyz: [0,0,0], rpy:[0,0,0] },
        geometry : { mesh : { filename : "meshes/abduction_mirror.stl" } },
        material : { color : { rgba : [0.75, 0.75, 0.75, 1] } }
      }
    },		
    "right_pelvis_rotation": {
      visual : { 
        origin : { xyz: [0,0,0], rpy:[0,0,0] },
        geometry : { mesh : { filename : "meshes/yaw_mirror.stl" } },
        material : { color : { rgba : [0.75, 0.75, 0.75, 1] } }
      }
    },			
    "right_hip": {
      visual : { 
        origin : { xyz: [0,0,0], rpy:[0,0,0] },
        geometry : { mesh : { filename : "meshes/hip_mirror.stl" } },
        material : { color : { rgba : [0.75, 0.75, 0.75, 1] } }
      }
    },		
    "right_thigh": {
      visual : { 
        origin : { xyz: [0,0,0], rpy:[0,0,0] },
        geometry : { mesh : { filename : "meshes/thigh_mirror.stl" } },
        material : { color : { rgba : [0.75, 0.75, 0.75, 1] } }
      }
    },		
    "right_knee": {
      visual : { 
        origin : { xyz: [0,0,0], rpy:[0,0,0] },
        geometry : { mesh : { filename : "meshes/knee-output_mirror.stl" } },
        material : { color : { rgba :  [0.35, 0.35, 0.35, 1] } }
      }
    },		
    "right_shin": {
      visual : { 
        origin : { xyz: [0,0,0], rpy:[0,0,0] },
        geometry : { mesh : { filename : "meshes/shin-bone_mirror.stl" } },
        material : { color : { rgba :  [0.35, 0.35, 0.35, 1] } }
      }
    },		
    "right_tarsus": {
      visual : { 
        origin : { xyz: [0,0,0], rpy:[0,0,0] },
        geometry : { mesh : { filename : "meshes/tarsus_mirror.stl" } },
        material : { color : { rgba :  [0.35, 0.35, 0.35, 1] } }
      }
    },	
    "right_toe": {
      visual : { 
        origin : { xyz: [0,0,0], rpy:[0,0,0] },
        geometry : { mesh : { filename : "meshes/toe_mirror.stl" } },
        material : { color : { rgba :  [0.75, 0.75, 0.75, 1]  } }
      }
    },		
  },
};

// specify name of endeffector frame
robot.endeffector = {};
robot.endeffector.frame = "fixed_left";
robot.endeffector.position = [[0.1],[0],[0],[1]]

robot.joints = {};

robot.joints.fixed_left = { 
    type : "fixed",
    parent: "pelvis", child: "left_pelvis_abduction", 
	axis:[1,0,0],
    origin : {xyz: [0.021, 0.135, 0], rpy:[0, 1.57079632679, 0]},
};
robot.joints.hip_abduction_left = { 
    type : "continuous",
    parent: "left_pelvis_abduction", child: "left_pelvis_rotation", 
    axis : [1,0,0],
    origin : {rpy: [0, -1.57079632679, 0], xyz:[0,0,-0.07]},
    //origin : {xyz: [0,0,0], rpy:[0,0,0]},
    limit : {lower:-0.2618, upper:0.3927}
};
robot.joints.hip_rotation_left = { 
    type : "continuous",
    parent: "left_pelvis_rotation", child: "left_hip", 
    axis : [-1,0,0],
    origin : {xyz: [0, 0, -0.09], rpy:[0, 1.57079632679, -1.57079632679]},
   // origin : {xyz: [0,0,0], rpy:[0,0,0]},
    limit : {lower:-0.3927, upper:0.3927}
};
robot.joints.hip_flexion_left = { 
    type : "continuous",
    parent: "left_hip", child: "left_thigh", 
    axis : [0,0,1],
    origin : {rpy: [0, 0, 0], xyz:[0, 0, 0]},
    limit : {lower:-0.8727, upper:1.3963}
};
robot.joints.knee_joint_left = { 
    type : "continuous",
    parent: "left_thigh", child: "left_knee", 
    axis : [0,0,1],
    origin : {rpy: [0, 0, 0], xyz:[0.12, 0, 0.0045]},
    limit : {lower:-2.8623, upper:-0.6458}
};
robot.joints.knee_to_shin_left = { 
    type : "continuous",
    parent: "left_knee", child: "left_shin", 
    axis : [0,0,1],
    origin : {rpy: [0, 0, 0], xyz:[0.0607, 0.0474, 0]},
    limit : {lower:-100, upper:100}
};
robot.joints.ankle_joint_left = { 
    type : "continuous",
    parent: "left_shin", child: "left_tarsus", 
    axis : [0,0,1],
    origin : {rpy: [0, 0, 0], xyz:[0.4348, 0.02, 0]},
    limit : {lower:-100, upper:100}
};
robot.joints.toe_joint_left = { 
    type : "continuous",
    parent: "left_tarsus", child: "left_toe", 
    axis : [0,0,1],
    origin : {rpy: [0, 0, 0], xyz:[0.408, -0.04, 0]},
    limit : {lower:-2.4435, upper:-0.5236}
};
robot.joints.fixed_right = { 
    type : "fixed",
    origin : {rpy: [0, 1.57079632679, 0], xyz:[0.021, -0.135, 0]},	
	axis:[1,0,0],
    parent: "pelvis", child: "right_pelvis_abduction", 
};
robot.joints.hip_abduction_right = { 
    type : "continuous",
    parent: "right_pelvis_abduction", child: "right_pelvis_rotation", 
    axis : [1,0,0],
    origin : {rpy: [0, -1.57079632679, 0], xyz:[0, 0, -0.07]},
    limit : {lower:-0.3927, upper:0.2618}
};
robot.joints.hip_rotation_right = { 
    type : "continuous",
	origin : {rpy: [0, 1.57079632679, -1.57079632679], xyz:[0, 0, -0.09]},
	axis : [-1,0,0],
    parent: "right_pelvis_rotation", child: "right_hip",     
    limit : {lower:-0.3927, upper:0.3927}
};
robot.joints.hip_flexion_right = { 
    type : "continuous",
    parent: "right_hip", child: "right_thigh", 
    axis : [0,0,1],
    origin : {rpy: [0, 0, 0], xyz:[0, 0, 0]},
    limit : {lower:-0.8727, upper:1.3963}
};
robot.joints.knee_joint_right = { 
    type : "continuous",
    parent: "right_thigh", child: "right_knee", 
    axis : [0,0,1],
    origin : {rpy: [0, 0, 0], xyz:[0.12, 0, -0.0045]},
    limit : {lower:-2.8623, upper:-0.6458}
};
robot.joints.knee_to_shin_right = { 
    type : "continuous",
    parent: "right_knee", child: "right_shin", 
    axis : [0,0,1],
    origin : {rpy: [0, 0, 0], xyz:[0.0607, 0.0474, 0]},
    limit : {lower:-100, upper:100}
};
robot.joints.ankle_joint_right = { 
    type : "continuous",
    parent: "right_shin", child: "right_tarsus", 
    axis : [0,0,1],
    origin : {rpy: [0, 0, 0], xyz:[0.4348, 0.02, 0]},
    limit : {lower:-100, upper:100}
};
robot.joints.toe_joint_right = { 
    type : "continuous",
    parent: "right_tarsus", child: "right_toe", 
    axis : [0,0,1],
    origin : {rpy: [0, 0, 0], xyz:[0.408, -0.04, 0]},
    limit : {lower:-2.4435, upper:-0.5236}
};

// note ROS coordinate system (x:forward, y:lateral, z:up) is different than threejs (x:lateral, y:up, z:forward)
robot.links_geom_imported = true;

links_geom = {};

  // KE: replace hardcoded robot directory
  // KE: replace file extension processing
i = 0;
for (x in robot.links) {
  //geom_index = robot.links[x].visual.geometry.mesh.filename.split('_adjusted')[0];
  //geom_extension = robot.links[x].visual.geometry.mesh.filename.split('_adjusted')[1];
  filename_split = robot.links[x].visual.geometry.mesh.filename.split('.');
  geom_index = filename_split[0];
  geom_extension = filename_split[filename_split.length-1];
  console.log(geom_index + "  " + geom_extension);
  //assignFetchModel('./robots/sawyer/'+robot.links[x].visual.geometry.mesh.filename,geom_index);
  if (geom_extension === "dae") { // extend to use regex
    assignFetchModelCollada('./robots/cassie_bnalin/'+robot.links[x].visual.geometry.mesh.filename,x);
  }
  else if (geom_extension === "DAE") { // extend to use regex
    assignFetchModelCollada('./robots/cassie_bnalin/'+robot.links[x].visual.geometry.mesh.filename,x);
  }
  else {
    assignFetchModelSTL('./robots/cassie_bnalin/'+robot.links[x].visual.geometry.mesh.filename,robot.links[x].visual.material,x);
  }
  i++;
}

function assignFetchModelCollada(filename,index) {

    console.log("assignFetchModel : "+filename+" - "+index); 
    var collada_loader = new THREE.ColladaLoader();
    var val = collada_loader.load(filename, 
       function ( collada ) {
            links_geom[index] = collada.scene;
        },
        function (xhr) {
            console.log(filename+" - "+index+": "+(xhr.loaded / xhr.total * 100) + '% loaded' );
        }
    );
}

function assignFetchModelCollada2(filename,index) {

    console.log("assignFetchModel : "+filename+" - "+index); 
    var collada_loader = new ColladaLoader2();
    var val = collada_loader.load(filename, 
       function ( collada ) {
            links_geom[index] = collada.scene;
        },
        function (xhr) {
            console.log(filename+" - "+index+": "+(xhr.loaded / xhr.total * 100) + '% loaded' );
        }
    );
}


function assignFetchModelSTL(filename,material_urdf,linkname) {

    console.log("assignFetchModel : "+filename+" - "+linkname); 
    var stl_loader = new THREE.STLLoader();
    var val = stl_loader.load(filename, 
       function ( geometry ) {
            // ocj: add transparency
            var material_color = new THREE.Color(material_urdf.color.rgba[0], material_urdf.color.rgba[1], material_urdf.color.rgba[2]);
            var material = new THREE.MeshLambertMaterial( {color: material_color, side: THREE.DoubleSide} );
            links_geom[linkname] = new THREE.Mesh( geometry, material ) ;
        } //,
        //function (xhr) {
        //    console.log(filename+" - "+linkname+": "+(xhr.loaded / xhr.total * 100) + '% loaded' );
        //}
    );
}
