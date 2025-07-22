# corrected_dae_converter.py
import bpy, os, sys, shutil, glob
import addon_utils
from datetime import datetime
import mathutils

def setup_blender_environment():
    """Setup Blender environment for mesh conversion"""
    try:
        # Clear scene first
        bpy.ops.wm.read_factory_settings(use_empty=True)
        
        # Set units to meters (standard for Gazebo Harmonic)
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

def analyze_mesh_properties(filepath):
    """Analyze the original mesh to understand its current properties"""
    try:
        # Clear scene
        bpy.ops.object.select_all(action='SELECT')
        bpy.ops.object.delete()
        
        # Import without any unit conversion first to analyze
        bpy.ops.wm.collada_import(
            filepath=filepath,
            import_units=False,  # Don't apply automatic scaling yet
            custom_normals=True,
            fix_orientation=False
        )
        
        # Get all mesh objects
        mesh_objects = [obj for obj in bpy.context.scene.objects if obj.type == 'MESH']
        
        if not mesh_objects:
            return None, None, None
        
        # Calculate bounding box of all objects
        min_coords = [float('inf')] * 3
        max_coords = [float('-inf')] * 3
        
        for obj in mesh_objects:
            bbox = [obj.matrix_world @ mathutils.Vector(corner) for corner in obj.bound_box]
            for corner in bbox:
                for i in range(3):
                    min_coords[i] = min(min_coords[i], corner[i])
                    max_coords[i] = max(max_coords[i], corner[i])
        
        dimensions = [max_coords[i] - min_coords[i] for i in range(3)]
        max_dimension = max(dimensions)
        
        # Check if Z-axis is the "up" axis by analyzing object orientations
        total_z_up = 0
        for obj in mesh_objects:
            # Get object's up vector in world space
            up_vector = obj.matrix_world.to_3x3() @ mathutils.Vector((0, 0, 1))
            total_z_up += abs(up_vector.z)
        
        avg_z_up = total_z_up / len(mesh_objects) if mesh_objects else 0
        is_z_up = avg_z_up > 0.7  # If Z component is dominant, it's Z-up
        
        print(f"  📐 Analysis: max_dim={max_dimension:.3f}, z_up_factor={avg_z_up:.2f}")
        
        return max_dimension, is_z_up, len(mesh_objects)
        
    except Exception as e:
        print(f"  ❌ Analysis failed: {e}")
        return None, None, None

def convert_mesh_smart(filepath, output_path):
    """Convert mesh with intelligent scaling and orientation detection"""
    
    print(f"  🔍 Analyzing mesh properties...")
    max_dim, is_z_up, obj_count = analyze_mesh_properties(filepath)
    
    if max_dim is None:
        print(f"  ❌ Could not analyze mesh")
        return False
    
    # Determine if scaling is needed
    # If max dimension > 10, likely in centimeters (scale down by 0.01)
    # If max dimension < 0.1, likely in millimeters (scale up by 1000)
    # Otherwise, likely already in meters
    
    if max_dim > 10.0:
        scale_factor = 0.01  # Convert cm to m
        print(f"  📏 Detected centimeters (max_dim={max_dim:.1f}), scaling by 0.01")
    elif max_dim < 0.01:
        scale_factor = 1000.0  # Convert mm to m  
        print(f"  📏 Detected millimeters (max_dim={max_dim:.4f}), scaling by 1000")
    else:
        scale_factor = 1.0  # Already in meters
        print(f"  📏 Detected meters (max_dim={max_dim:.3f}), no scaling needed")
    
    # Determine if rotation is needed
    needs_rotation = not is_z_up
    print(f"  🧭 Z-up orientation: {'✅ correct' if is_z_up else '❌ needs rotation'}")
    
    # Clear and re-import for processing
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete()
    
    # Import with appropriate settings
    bpy.ops.wm.collada_import(
        filepath=filepath,
        import_units=False,  # Handle scaling manually for precision
        custom_normals=True,
        fix_orientation=False
    )
    
    # Get all mesh objects
    mesh_objects = [obj for obj in bpy.context.scene.objects if obj.type == 'MESH']
    
    if not mesh_objects:
        print(f"  ❌ No mesh objects found after import")
        return False
    
    # Select all mesh objects
    bpy.ops.object.select_all(action='DESELECT')
    for obj in mesh_objects:
        obj.select_set(True)
    bpy.context.view_layer.objects.active = mesh_objects[0]
    
    # Apply scaling if needed
    if scale_factor != 1.0:
        bpy.ops.transform.resize(value=(scale_factor, scale_factor, scale_factor))
        print(f"  📐 Applied scale factor: {scale_factor}")
    
    # Apply rotation if needed (Y-up to Z-up)
    if needs_rotation:
        bpy.ops.transform.rotate(value=1.5708, orient_axis='X')  # 90° around X
        print(f"  🔄 Applied Y-up to Z-up rotation")
    
    # Apply transformations
    bpy.ops.object.transform_apply(location=False, rotation=True, scale=True)
    
    # Export with proper settings
    export_base = output_path[:-4] if output_path.endswith('.dae') else output_path
    
    try:
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
            # Fix unit specification in exported file
            fix_dae_units(exported_file)
            print(f"  ✅ Exported successfully")
            return exported_file
        else:
            print(f"  ❌ Export failed - file not created")
            return False
            
    except Exception as e:
        print(f"  ❌ Export failed: {e}")
        return False

def fix_dae_units(filepath):
    """Fix unit specification in DAE file for Gazebo Harmonic"""
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            content = f.read()
        
        import re
        
        # Replace any unit specification with meters
        patterns = [
            (r'<unit[^>]*meter="[^"]*"[^>]*name="[^"]*"[^>]*/>', '<unit meter="1.0" name="meter"/>'),
            (r'<unit[^>]*name="[^"]*"[^>]*meter="[^"]*"[^>]*/>', '<unit name="meter" meter="1.0"/>'),
        ]
        
        for pattern, replacement in patterns:
            content = re.sub(pattern, replacement, content)
        
        # Ensure there's at least one unit specification
        if '<unit ' not in content and '<asset>' in content:
            content = content.replace('<asset>', '<asset>\n    <unit name="meter" meter="1.0"/>')
        
        with open(filepath, 'w', encoding='utf-8') as f:
            f.write(content)
        
        print(f"  🔧 Fixed unit specification")
        return True
        
    except Exception as e:
        print(f"  ⚠️  Could not fix units: {e}")
        return False

def process_single_dae(input_path, backup_dir=None):
    """Process a single DAE file"""
    print(f"🔄 Processing: {os.path.basename(input_path)}")
    
    # Create backup if requested
    if backup_dir:
        backup_path = os.path.join(backup_dir, os.path.basename(input_path))
        shutil.copy2(input_path, backup_path)
        print(f"  💾 Backup created")
    
    # Create temporary file
    temp_path = input_path + '.tmp'
    
    try:
        # Convert mesh
        result = convert_mesh_smart(input_path, temp_path)
        
        if result and os.path.exists(result):
            # Replace original with converted version
            shutil.move(result, input_path)
            print(f"  ✅ Conversion successful")
            return True
        else:
            print(f"  ❌ Conversion failed")
            return False
            
    except Exception as e:
        print(f"  ❌ Error: {e}")
        return False
    finally:
        # Clean up temporary files
        for tmp_file in [temp_path, temp_path + '.dae']:
            if os.path.exists(tmp_file):
                os.remove(tmp_file)

def batch_convert_meshes(input_dir, create_backup=True):
    """Batch convert all DAE files in directory"""
    
    if not setup_blender_environment():
        return False
    
    # Find all DAE files
    dae_files = []
    for root, dirs, files in os.walk(input_dir):
        for f in files:
            if f.lower().endswith('.dae') and not '.tmp' in f:
                dae_files.append(os.path.join(root, f))
    
    print(f"📋 Found {len(dae_files)} DAE files to process")
    
    # Create backup directory
    backup_dir = None
    if create_backup:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        backup_dir = f"{input_dir}_backup_{timestamp}"
        os.makedirs(backup_dir, exist_ok=True)
        print(f"📁 Backup directory: {backup_dir}")
    
    # Process files
    successful = 0
    failed = 0
    
    for dae_file in dae_files:
        if process_single_dae(dae_file, backup_dir):
            successful += 1
        else:
            failed += 1
        print()
    
    print("📊 Conversion Summary:")
    print(f"  ✅ Successful: {successful}")
    print(f"  ❌ Failed: {failed}")
    print(f"  📁 Total files: {len(dae_files)}")
    
    return failed == 0

if __name__ == "__main__":
    if len(sys.argv) < 4:
        print("Usage: blender --background --python corrected_dae_converter.py -- <meshes_directory> [--no-backup]")
        print("Example: blender --background --python corrected_dae_converter.py -- blimp_description/meshes")
        sys.exit(1)
    
    # Parse arguments
    args = sys.argv[sys.argv.index("--") + 1:]
    input_dir = args[0]
    create_backup = "--no-backup" not in args
    
    if not os.path.exists(input_dir):
        print(f"❌ ERROR: Directory '{input_dir}' does not exist")
        sys.exit(1)
    
    print("🚀 Smart DAE Converter for Gazebo Harmonic")
    print("=" * 50)
    print(f"📂 Input directory: {input_dir}")
    print(f"💾 Create backup: {create_backup}")
    print()
    
    success = batch_convert_meshes(input_dir, create_backup)
    
    if success:
        print("🎉 All meshes converted successfully!")
        sys.exit(0)
    else:
        print("⚠️  Some conversions failed. Check output above.")
        sys.exit(1)