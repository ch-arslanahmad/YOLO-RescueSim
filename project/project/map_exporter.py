#!/usr/bin/env python3
"""
Map Visualization and Export Tool
Converts tracked humans into a visual map and exports different formats
"""

import json
import csv
from pathlib import Path
from datetime import datetime
import numpy as np


class HumanMapExporter:
    def __init__(self, output_dir='/home/arslan/Desktop/github/YOLO-RescueSim/project/human_detections'):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)
    
    def export_to_formats(self, tracked_humans, filename_prefix='human_map'):
        """Export tracking data to multiple formats"""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        # CSV Export
        csv_file = self.output_dir / f'{filename_prefix}_{timestamp}.csv'
        self._export_csv(tracked_humans, csv_file)
        
        # JSON Export
        json_file = self.output_dir / f'{filename_prefix}_{timestamp}.json'
        self._export_json(tracked_humans, json_file)
        
        # TXT Report
        txt_file = self.output_dir / f'{filename_prefix}_{timestamp}.txt'
        self._export_report(tracked_humans, txt_file)
        
        # ASCII Map
        ascii_file = self.output_dir / f'{filename_prefix}_{timestamp}_ascii.txt'
        self._export_ascii_map(tracked_humans, ascii_file)
        
        return {
            'csv': csv_file,
            'json': json_file,
            'report': txt_file,
            'ascii_map': ascii_file
        }
    
    def _export_csv(self, tracked_humans, filepath):
        """Export as CSV"""
        with open(filepath, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Human_ID', 'X_Position_m', 'Y_Position_m', 'Detections_Count', 'Confidence'])
            
            for human_id, data in tracked_humans.items():
                pos = data.get('position', [0, 0])
                writer.writerow([
                    human_id,
                    f"{pos[0]:.2f}",
                    f"{pos[1]:.2f}",
                    len(data.get('history', [])),
                    f"{data.get('confidence', 0):.2f}"
                ])
        print(f"CSV exported to {filepath}")
    
    def _export_json(self, tracked_humans, filepath):
        """Export as JSON with full details"""
        export_data = {
            'metadata': {
                'timestamp': datetime.now().isoformat(),
                'total_humans': len(tracked_humans),
                'export_format': 'human_tracking_v1'
            },
            'humans': tracked_humans
        }
        
        with open(filepath, 'w') as f:
            json.dump(export_data, f, indent=2)
        print(f"JSON exported to {filepath}")
    
    def _export_report(self, tracked_humans, filepath):
        """Export human-readable report"""
        with open(filepath, 'w') as f:
            f.write("="*60 + "\n")
            f.write("RESCUE SIMULATION - HUMAN DETECTION REPORT\n")
            f.write("="*60 + "\n")
            f.write(f"Generated: {datetime.now().isoformat()}\n\n")
            
            f.write(f"Total Humans Detected: {len(tracked_humans)}\n\n")
            
            f.write("HUMAN LOCATIONS:\n")
            f.write("-" * 60 + "\n")
            
            for human_id, data in sorted(tracked_humans.items()):
                pos = data.get('position', [0, 0])
                f.write(f"\nHuman ID: {human_id}\n")
                f.write(f"  Position (X, Y): ({pos[0]:.2f}m, {pos[1]:.2f}m)\n")
                f.write(f"  Times Detected: {len(data.get('history', []))}\n")
                f.write(f"  Confidence: {data.get('confidence', 0):.2f}\n")
                
                history = data.get('history', [])
                if history:
                    f.write(f"  First Detection: {history[0].get('timestamp', 'N/A')}\n")
                    f.write(f"  Last Detection: {history[-1].get('timestamp', 'N/A')}\n")
        
        print(f"Report exported to {filepath}")
    
    def _export_ascii_map(self, tracked_humans, filepath):
        """Export as ASCII visualization"""
        # Create grid representation
        grid_size = 50
        grid = [['.' for _ in range(grid_size)] for _ in range(grid_size)]
        
        # Scale factor (adjust based on field size)
        scale = 2.0  # pixels per meter
        center_x, center_y = grid_size // 2, grid_size // 2
        
        # Plot humans on grid
        with open(filepath, 'w') as f:
            f.write("RESCUE FIELD MAP - ASCII VISUALIZATION\n")
            f.write("="*60 + "\n\n")
            f.write("H = Detected Human\n")
            f.write(". = Empty Space\n")
            f.write("~ = Origin/Robot Start Area\n\n")
            
            # Place robot at center
            if 0 <= center_y < grid_size and 0 <= center_x < grid_size:
                grid[center_y][center_x] = '~'
            
            # Place humans
            for human_id, data in tracked_humans.items():
                pos = data.get('position', [0, 0])
                
                # Convert to grid coordinates
                grid_x = int(center_x + pos[0] * scale)
                grid_y = int(center_y + pos[1] * scale)
                
                # Bound check and place
                if 0 <= grid_x < grid_size and 0 <= grid_y < grid_size:
                    grid[grid_y][grid_x] = 'H'
            
            # Write grid
            for row in grid:
                f.write(''.join(row) + '\n')
            
            f.write("\n" + "="*60 + "\n")
            f.write("COORDINATES:\n")
            for human_id, data in tracked_humans.items():
                pos = data.get('position', [0, 0])
                f.write(f"Human {human_id}: X={pos[0]:.2f}m, Y={pos[1]:.2f}m\n")
        
        print(f"ASCII map exported to {filepath}")


def main():
    """Test exporter"""
    # Example tracked humans data
    example_data = {
        '1': {
            'position': [2.5, 3.0],
            'confidence': 0.95,
            'history': [
                {'position': [2.4, 2.9], 'confidence': 0.92, 'timestamp': '2024-01-04T10:00:00'},
                {'position': [2.5, 3.0], 'confidence': 0.95, 'timestamp': '2024-01-04T10:00:05'}
            ]
        },
        '2': {
            'position': [-3.0, -2.5],
            'confidence': 0.88,
            'history': [
                {'position': [-2.9, -2.4], 'confidence': 0.85, 'timestamp': '2024-01-04T10:00:02'}
            ]
        }
    }
    
    exporter = HumanMapExporter()
    files = exporter.export_to_formats(example_data, 'test_map')
    
    print("\nExport complete!")
    print(f"Files created: {list(files.values())}")


if __name__ == '__main__':
    main()
