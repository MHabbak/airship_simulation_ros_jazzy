# batch_dae_convert_corrected.py
import bpy, os, sys, shutil, glob
import addon_utils
from datetime import datetime

def setup_blender_environment():
    """Setup Blender environment and enable required add-ons"""
    try:
        # Ensure Blender uses meters as base unit
        scene = bpy.context.scene
        scene.unit_settings.system = 'METRIC'
        scene.unit_settings.scale_length = 1.0  # 1 Blender Unit = 1 meter

        addon_utils.enable("io_scene_collada", default_set=True, persistent=True)
        print("✅ COLLADA add-on enabled")
        bpy.utils.refresh_script_paths()
        return True
    except Exception as e:
        print(f"❌ Failed to setup Blender environment: {e}")
        return False

def cleanup_temp_files(input_dir):
    """Clean up any leftover temporary files from previous runs"""
    print("🧹 Cleaning up temporary files from previous runs...")
    
    temp_patterns = [
        "*.tmp.dae",
        "*.tmp.tmp.dae", 
        "*.tmp.tmp.tmp.dae",
        "*.dae.tmp.dae",
        "*_processing_temp.dae"
    ]
    
    cleaned_count = 0
    for root, dirs, files in os.walk(input_dir):
        for pattern in temp_patterns:
            temp_files = glob.glob(os.path.join(root, pattern))
            for temp_file in temp_files:
                try:
                    os.remove(temp_file)
                    print(f"  🗑️  Removed: {os.path.basename(temp_file)}")
                    cleaned_count += 1
                except Exception as e:
                    print(f"  ⚠️  Could not remove {temp_file}: {e}")
    
    if cleaned_count > 0:
        print(f"✅ Cleaned up {cleaned_count} temporary files")
    else:
        print("✅ No temporary files found to clean")
    print()

def is_original_dae_file(filepath):
    """Check if file is an original DAE file (not a temporary one)"""
    basename = os.path.basename(filepath)
    
    # Skip files with .tmp in the name
    if '.tmp' in basename:
        return False
    
    # Skip processing temp files
    if '_processing_temp' in basename:
        return False
    
    # Only process files ending in .dae
    if not basename.lower().endswith('.dae'):
        return False
    
    # Skip files that look like temporary files
    temp_indicators = ['.temp.', '.backup.']
    for indicator in temp_indicators:
        if indicator in basename.lower():
            return False
    
    return True

def clear_scene():
    """Clear scene using factory settings"""
    bpy.ops.wm.read_factory_settings(use_empty=True)

def import_dae(filepath):
    """Import DAE file with automatic unit conversion"""
    try:
        bpy.ops.wm.collada_import(
            filepath=filepath,
            import_units=True,        # CRITICAL: Blender handles cm→m conversion automatically
            custom_normals=True,
            fix_orientation=False
        )
        print(f"  ✅ Imported successfully (units auto-converted)")
        return True
    except Exception as e:
        print(f"  ❌ Import failed: {e}")
        return False

def fix_y_to_z_orientation():
    """Convert Y-UP to Z-UP orientation ONLY (no scaling)"""
    mesh_objects = [obj for obj in bpy.context.scene.objects if obj.type == 'MESH']
    
    if not mesh_objects:
        print("  ⚠️  No mesh objects found to rotate")
        return False
    
    # Select all mesh objects
    bpy.ops.object.select_all(action='DESELECT')
    for obj in mesh_objects:
        obj.select_set(True)
        bpy.context.view_layer.objects.active = obj
    
    try:
        # ONLY rotate from Y-UP to Z-UP (NO manual scaling!)
        # Blender already handled unit conversion during import
        bpy.ops.transform.rotate(value=1.5708, orient_axis='X')  # 90° around X
        bpy.ops.object.transform_apply(rotation=True, scale=True)
        print(f"  ✅ Rotated {len(mesh_objects)} mesh objects (Y-UP → Z-UP)")
        return True
    except Exception as e:
        print(f"  ❌ Rotation failed: {e}")
        return False

def export_dae(output_filepath):
    """Export DAE file with proper units and orientation"""
    
    # Remove .dae extension if present, Blender adds it automatically
    if output_filepath.endswith('.dae'):
        export_path = output_filepath[:-4]
    else:
        export_path = output_filepath
    
    try:
        bpy.ops.wm.collada_export(
            filepath=export_path,
            apply_modifiers=True,
            triangulate=False,
            use_texture_copies=True,
            apply_global_orientation=True,
            export_global_up_selection='Z',      # Export as Z-UP
            export_global_forward_selection='Y', # Forward axis
            export_mesh_type=0,
            selected=False,
            include_children=True,
            include_armatures=False,
            include_shapekeys=False,
            # Unit settings will be handled by Blender's scene units
        )
        
        # The actual exported file will have .dae extension
        actual_file = export_path + '.dae'
        
        if os.path.exists(actual_file):
            print(f"  ✅ Exported with Z-UP orientation and meter units")
            return actual_file
        else:
            print(f"  ❌ Export failed - file not created: {actual_file}")
            return None
            
    except Exception as e:
        print(f"  ❌ Export failed: {e}")
        return None

def fix_exported_dae_units(filepath):
    """Fix the unit specification in the exported DAE file"""
    try:
        # Read the file
        with open(filepath, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # Replace various unit specifications with meter
        import re
        
        # Pattern to match unit tags with different formats
        patterns = [
            (r'<unit\s+meter="[^"]*"\s+name="(centimeter|inch|millimeter|cm|mm)"\s*/>', 
             '<unit meter="1.0" name="meter"/>'),
            (r'<unit\s+name="(centimeter|inch|millimeter|cm|mm)"\s+meter="[^"]*"\s*/>', 
             '<unit name="meter" meter="1.0"/>'),
        ]
        
        for pattern, replacement in patterns:
            content = re.sub(pattern, replacement, content)
        
        # Write back the corrected file
        with open(filepath, 'w', encoding='utf-8') as f:
            f.write(content)
        
        print(f"  ✅ Fixed unit specification to meters")
        return True
        
    except Exception as e:
        print(f"  ⚠️  Could not fix units in file: {e}")
        return False

def process_dae_inplace(filepath, backup_dir=None):
    """Process single DAE file in-place"""
    print(f"Processing: {os.path.basename(filepath)}")
    
    # Verify this is an original DAE file
    if not is_original_dae_file(filepath):
        print(f"  ⏭️  Skipping temporary/invalid file")
        return True  # Don't count as failure
    
    # Create backup if backup directory specified
    if backup_dir:
        backup_path = os.path.join(backup_dir, os.path.basename(filepath))
        shutil.copy2(filepath, backup_path)
        print(f"  💾 Backup created")
    
    # Create unique temporary filename
    base_dir = os.path.dirname(filepath)
    base_name = os.path.splitext(os.path.basename(filepath))[0]
    temp_filename = f"{base_name}_processing_temp"
    temp_path = os.path.join(base_dir, temp_filename)
    
    try:
        # Step 1: Clear scene and set units
        clear_scene()
        
        # Step 2: Import DAE (automatic unit conversion)
        if not import_dae(filepath):
            raise Exception("Failed to import DAE file")
        
        # Step 3: Fix orientation ONLY (no manual scaling)
        if not fix_y_to_z_orientation():
            raise Exception("Failed to fix orientation")
        
        # Step 4: Export to temporary file
        exported_file = export_dae(temp_path)
        if not exported_file or not os.path.exists(exported_file):
            raise Exception(f"Export failed - file not created")
        
        # Step 5: Fix unit specification in exported file
        fix_exported_dae_units(exported_file)
        
        # Step 6: Replace original with processed version
        shutil.move(exported_file, filepath)
        print(f"  ✅ Successfully converted")
        return True
        
    except Exception as e:
        print(f"  ❌ ERROR: {e}")
        
        # Clean up any temporary files
        cleanup_patterns = [temp_path + '.dae', temp_path + '.tmp.dae']
        for cleanup_file in cleanup_patterns:
            if os.path.exists(cleanup_file):
                try:
                    os.remove(cleanup_file)
                except:
                    pass
        
        return False

def batch_process_inplace(input_dir, create_backup=True):
    """Process all original DAE files in directory tree"""
    
    print("🔧 Setting up Blender environment...")
    if not setup_blender_environment():
        print("❌ Failed to setup Blender environment")
        return False
    
    # Clean up temporary files from previous runs
    cleanup_temp_files(input_dir)
    
    # Create backup directory if requested
    backup_dir = None
    if create_backup:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        backup_dir = f"{input_dir}_backup_final_{timestamp}"
        os.makedirs(backup_dir, exist_ok=True)
        print(f"📁 Backup directory created: {backup_dir}")
    
    # Find all ORIGINAL DAE files (excluding temporary ones)
    original_dae_files = []
    for root, _, files in os.walk(input_dir):
        for f in files:
            filepath = os.path.join(root, f)
            if is_original_dae_file(filepath):
                original_dae_files.append(filepath)
    
    print(f"📋 Found {len(original_dae_files)} original DAE files to process")
    print()
    
    # Process each file
    successful = 0
    failed = 0
    
    for filepath in original_dae_files:
        result = process_dae_inplace(filepath, backup_dir)
        if result:
            successful += 1
        else:
            failed += 1
        print()  # Empty line for readability
    
    print("📊 Processing Summary:")
    print(f"  ✅ Successful: {successful}")
    print(f"  ❌ Failed: {failed}")
    print(f"  📁 Total original files: {len(original_dae_files)}")
    
    if backup_dir:
        print(f"  💾 Backups saved to: {backup_dir}")
    
    return failed == 0

if __name__ == "__main__":
    if len(sys.argv) < 4:
        print("Usage: blender --background --python batch_dae_convert_corrected.py -- <meshes_directory> [--no-backup]")
        print("Example: blender --background --python batch_dae_convert_corrected.py -- meshes")
        sys.exit(1)
    
    # Parse arguments
    args = sys.argv[sys.argv.index("--") + 1:]
    input_dir = args[0]
    create_backup = "--no-backup" not in args
    
    if not os.path.exists(input_dir):
        print(f"❌ ERROR: Directory '{input_dir}' does not exist")
        sys.exit(1)
    
    print("🚀 DAE Z-UP Conversion Script (Corrected - No Double Scaling)")
    print("=" * 70)
    print(f"📂 Input directory: {input_dir}")
    print(f"💾 Create backup: {create_backup}")
    print()
    
    success = batch_process_inplace(input_dir, create_backup)
    
    if success:
        print("🎉 All files processed successfully!")
        print("✅ Your DAE files now have Z-UP orientation, correct scale, and meter units!")
        sys.exit(0)
    else:
        print("⚠️  Some files failed to process. Check the output above.")
        sys.exit(1)