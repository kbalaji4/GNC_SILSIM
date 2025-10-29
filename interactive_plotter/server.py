#!/usr/bin/env python3
"""
Simple HTTP server to serve the interactive plotter
Run this script and open http://localhost:8000 in your browser
"""

import http.server
import socketserver
import os
import webbrowser
from pathlib import Path

def main():
    # Change to the parent directory to serve both plotter files and output data
    parent_dir = Path(__file__).parent.parent
    os.chdir(parent_dir)
    
    # Create a simple HTTP server
    PORT = 8000
    
    class CustomHTTPRequestHandler(http.server.SimpleHTTPRequestHandler):
        def end_headers(self):
            # Add CORS headers to allow loading CSV files
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
            self.send_header('Access-Control-Allow-Headers', 'Content-Type')
            super().end_headers()
    
    with socketserver.TCPServer(("", PORT), CustomHTTPRequestHandler) as httpd:
        print(f"🚀 EKF Interactive Plotter Server")
        print(f"📊 Serving at: http://localhost:{PORT}")
        print(f"📁 Directory: {parent_dir}")
        print(f"🌐 Opening browser...")
        print(f"⏹️  Press Ctrl+C to stop the server")
        print("-" * 50)
        
        # Open browser automatically to the interactive plotter
        webbrowser.open(f'http://localhost:{PORT}/interactive_plotter/')
        
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\n🛑 Shutting down server...")
            httpd.shutdown()
            print("👋 Server stopped. Goodbye!")

if __name__ == "__main__":
    main()
