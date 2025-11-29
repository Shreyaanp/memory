"""
Main script to process all images in the photos folder for spoof detection.
"""

import os
from pathlib import Path
from spoofservice import SpoofService


def get_image_files(folder_path: str) -> list:
    """Get all image files from the specified folder."""
    image_extensions = {'.png', '.jpg', '.jpeg', '.bmp', '.gif', '.webp'}
    folder = Path(folder_path)
    
    if not folder.exists():
        print(f"Error: Folder '{folder_path}' does not exist")
        return []
    
    image_files = [
        f for f in folder.iterdir() 
        if f.is_file() and f.suffix.lower() in image_extensions
    ]
    
    return sorted(image_files)


def main():
    # Initialize the spoof detection service
    print("Initializing Spoof Detection Service...")
    try:
        service = SpoofService()
    except ValueError as e:
        print(f"Error: {e}")
        return
    
    # Get the photos folder path
    script_dir = Path(__file__).parent
    photos_folder = script_dir / "photos"
    
    # Get all image files
    image_files = get_image_files(photos_folder)
    
    if not image_files:
        print("No image files found in the photos folder.")
        return
    
    print(f"Found {len(image_files)} images to process.\n")
    print("-" * 50)
    
    # Store results in memory
    results = {}
    
    # Process each image
    for image_path in image_files:
        filename = image_path.name
        print(f"Processing: {filename}...", end=" ")
        
        # Call the spoof detection service
        response = service.detect_spoof_from_file(str(image_path))
        
        # Store result
        results[filename] = response
        
        if response['success']:
            print(f"✓ {response['result']}")
        else:
            print(f"✗ Error: {response.get('error', 'Unknown error')}")
    
    # Print summary
    print("-" * 50)
    print("\n=== RESULTS SUMMARY ===\n")
    
    for filename, response in results.items():
        if response['success']:
            # Show all concepts with their confidence values
            concepts_str = ", ".join(
                [f"{c['name']}: {c['value']:.2%}" for c in response['concepts']]
            )
            print(f"{filename}: {response['result']} ({concepts_str})")
        else:
            print(f"{filename}: ERROR - {response.get('error', 'Unknown error')}")


if __name__ == "__main__":
    main()

