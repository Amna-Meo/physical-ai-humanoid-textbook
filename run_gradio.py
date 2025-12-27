#!/usr/bin/env python
"""
Hugging Face Space launch script
This script is used by Hugging Face Spaces to start the application
"""
import os
from app import gradio_interface

if __name__ == "__main__":
    # Launch the Gradio interface
    gradio_interface.launch(
        server_name="0.0.0.0",
        server_port=int(os.getenv("PORT", 7860)),
        share=False,  # Don't create a public URL
        show_error=True
    )