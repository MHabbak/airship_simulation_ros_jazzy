import bpy, os, sys, shutil, glob
import addon_utils
from datetime import datetime

def setup_blender_environment():
    """Setup Blender environment"""
    try:
        # Clear scene first
        bpy.ops.wm.read_factory_settings(use_empty=True)
        
        # Set units to meters
        scene = bpy.context.scene
        scene.unit_settings.system = 'METRIC'
        scene.unit_settings.length_unit = 'METERS'
        scene.unit_settings.scale_length = 1.0
        
        addon_utils.enable("io_scene_collada", default_set=True, persistent=True)
        print("✅ COLLADA add-on enabled")
        bpy.utils.refresh_script_paths()
        return True
    except Exception as e:
        print(f"❌ Failed to setup Blender environment: {e}")
        return False

def is_blimp_body_file(filepath):
    """Check if this is a blimp body DAE file"""
    filename = os.path.basename(filepath).lower()
    return filename.startswith('blimp_body') and filename.endswith('.dae')

def rotate_blimp_body_mesh(filepath, output_path):
    """Apply 90° counterclockwise Y-rotation to blimp body mesh (around length axis)"""
    try:
        # Clear scene
        bpy.ops.object.select_all(action='SELECT')
        bpy.ops.object.delete()
        
        # Import the mesh
        bpy.ops.wm.collada_import(
            filepath=filepath,
            import_units=False,  # Don't change scaling
            custom_normals=True,
            fix_orientation=False
        )
        
        # Get all mesh objects
        mesh_objects = [obj for obj in bpy.context.scene.objects if obj.type == 'MESH']
        
        if not mesh_objects:
            print(f"  ❌ No mesh objects found")
            return False
        
        # Select all mesh objects
        bpy.ops.object.select_all(action='DESELECT')
        for obj in mesh_objects:
            obj.select_set(True)
        bpy.context.view_layer.objects.active = mesh_objects[0]
        
        # Apply 90° counterclockwise rotation around Y-axis (blimp's long axis)
        bpy.ops.transform.rotate(value=1.5708, orient_axis='X')  # π/2 radians = 90°
        print(f"  🔄 Applied 90° Y-axis rotation (counterclockwise around length axis)")
        
        # Apply transformations
        bpy.ops.object.transform_apply(location=False, rotation=True, scale=True)
        
        # Export
        export_base = output_path[:-4] if output_path.endswith('.dae') else output_path
        
        bpy.ops.wm.collada_export(
            filepath=export_base,
            apply_modifiers=True,
            triangulate=False,
            use_texture_copies=True,
            apply_global_orientation=True,
            export_global_up_selection='Z',
            export_global_forward_selection='Y',
            export_mesh_type=0,
            selected=False,
            include_children=True,
            include_armatures=False,
            include_shapekeys=False,
        )
        
        exported_file = export_base + '.dae'
        
        if os.path.exists(exported_file):
            print(f"  ✅ Export successful")
            return exported_file
        else:
            print(f"  ❌ Export failed")
            return False
            
    except Exception as e:
        print(f"  ❌ Error: {e}")
        return False

def process_single_blimp_body(input_path, backup_dir=None):
    """Process a single blimp body DAE file"""
    print(f"🎯 Processing blimp body: {os.path.basename(input_path)}")
    
    # Create backup if requested
    if backup_dir:
        backup_path = os.path.join(backup_dir, os.path.basename(input_path))
        shutil.copy2(input_path, backup_path)
        print(f"  💾 Backup created")
    
    # Create temporary file
    temp_path = input_path + '.tmp'
    
    try:
        # Rotate mesh
        result = rotate_blimp_body_mesh(input_path, temp_path)
        
        if result and os.path.exists(result):
            # Replace original with rotated version
            shutil.move(result, input_path)
            print(f"  ✅ Rotation successful")
            return True
        else:
            print(f"  ❌ Rotation failed")
            return False
            
    except Exception as e:
        print(f"  ❌ Error: {e}")
        return False
    finally:
        # Clean up temporary files
        for tmp_file in [temp_path, temp_path + '.dae']:
            if os.path.exists(tmp_file):
                os.remove(tmp_file)

def rotate_all_blimp_bodies(input_dir, create_backup=True):
    """Find and rotate all blimp body meshes"""
    
    if not setup_blender_environment():
        return False
    
    # Find all blimp body DAE files
    blimp_body_files = []
    for root, dirs, files in os.walk(input_dir):
        for f in files:
            filepath = os.path.join(root, f)
            if is_blimp_body_file(filepath):
                blimp_body_files.append(filepath)
    
    if not blimp_body_files:
        print("❌ No blimp body files found!")
        return False
    
    print(f"📋 Found {len(blimp_body_files)} blimp body files:")
    for f in blimp_body_files:
        print(f"  - {os.path.basename(f)}")
    print()
    
    # Create backup directory
    backup_dir = None
    if create_backup:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        backup_dir = f"{input_dir}_blimp_backup_{timestamp}"
        os.makedirs(backup_dir, exist_ok=True)
        print(f"📁 Backup directory: {backup_dir}")
    
    # Process files
    successful = 0
    failed = 0
    
    for blimp_file in blimp_body_files:
        if process_single_blimp_body(blimp_file, backup_dir):
            successful += 1
        else:
            failed += 1
        print()
    
    print("📊 Rotation Summary:")
    print(f"  ✅ Successful: {successful}")
    print(f"  ❌ Failed: {failed}")
    print(f"  🎯 Total blimp body files: {len(blimp_body_files)}")
    
    return failed == 0

if __name__ == "__main__":
    if len(sys.argv) < 4:
        print("Usage: blender --background --python blimp_body_rotator.py -- <meshes_directory> [--no-backup]")
        print("Example: blender --background --python blimp_body_rotator.py -- blimp_description/meshes")
        sys.exit(1)
    
    # Parse arguments
    args = sys.argv[sys.argv.index("--") + 1:]
    input_dir = args[0]
    create_backup = "--no-backup" not in args
    
    if not os.path.exists(input_dir):
        print(f"❌ ERROR: Directory '{input_dir}' does not exist")
        sys.exit(1)
    
    print("🎯 Blimp Body Z-Rotation Script")
    print("=" * 40)
    print(f"📂 Input directory: {input_dir}")
    print(f"💾 Create backup: {create_backup}")
    print("🔄 Rotation: 90° counterclockwise around Y-axis (length axis)")
    print()
    
    success = rotate_all_blimp_bodies(input_dir, create_backup)
    
    if success:
        print("🎉 All blimp body meshes rotated successfully!")
        sys.exit(0)
    else:
        print("⚠️  Some rotations failed. Check output above.")
        sys.exit(1)