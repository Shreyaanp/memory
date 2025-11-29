"""
Spoof Detection Service using Clarifai Silent Face Anti-Spoofing API
"""

import os
from dotenv import load_dotenv
from clarifai_grpc.channel.clarifai_channel import ClarifaiChannel
from clarifai_grpc.grpc.api import resources_pb2, service_pb2, service_pb2_grpc
from clarifai_grpc.grpc.api.status import status_code_pb2

# Load environment variables
load_dotenv()


class SpoofService:
    """Service class for face anti-spoofing detection."""
    
    def __init__(self):
        self.pat = os.getenv('PAT')
        self.user_id = os.getenv('USER_ID', 'minivision-ai')
        self.app_id = os.getenv('APP_ID', 'silent-face-anti-spoofing')
        self.model_id = os.getenv('MODEL_ID', 'silent-face-anti-spoofing')
        self.model_version_id = os.getenv('MODEL_VERSION_ID', '727e809518234b9e8778e7ee16c74e96')
        
        if not self.pat:
            raise ValueError("PAT (Personal Access Token) not found in environment variables")
        
        # Setup gRPC channel and stub
        self.channel = ClarifaiChannel.get_grpc_channel()
        self.stub = service_pb2_grpc.V2Stub(self.channel)
        self.metadata = (('authorization', 'Key ' + self.pat),)
        self.user_data_object = resources_pb2.UserAppIDSet(
            user_id=self.user_id, 
            app_id=self.app_id
        )
    
    def detect_spoof_from_file(self, image_path: str) -> dict:
        """
        Detect if an image is a spoof (fake face) or real.
        
        Args:
            image_path: Path to the image file
            
        Returns:
            dict with 'success', 'result', and 'concepts' keys
        """
        try:
            # Read image bytes
            with open(image_path, 'rb') as f:
                image_bytes = f.read()
            
            # Make API request
            response = self.stub.PostModelOutputs(
                service_pb2.PostModelOutputsRequest(
                    user_app_id=self.user_data_object,
                    model_id=self.model_id,
                    version_id=self.model_version_id,
                    inputs=[
                        resources_pb2.Input(
                            data=resources_pb2.Data(
                                image=resources_pb2.Image(
                                    base64=image_bytes
                                )
                            )
                        )
                    ]
                ),
                metadata=self.metadata
            )
            
            # Check response status
            if response.status.code != status_code_pb2.SUCCESS:
                return {
                    'success': False,
                    'error': response.status.description,
                    'result': None,
                    'concepts': []
                }
            
            # Parse output
            output = response.outputs[0]
            concepts = []
            
            for concept in output.data.concepts:
                concepts.append({
                    'name': concept.name,
                    'value': concept.value
                })
            
            # Determine the primary result (highest confidence)
            if concepts:
                top_concept = max(concepts, key=lambda x: x['value'])
                result = top_concept['name']
            else:
                result = 'unknown'
            
            return {
                'success': True,
                'result': result,
                'concepts': concepts
            }
            
        except FileNotFoundError:
            return {
                'success': False,
                'error': f'Image file not found: {image_path}',
                'result': None,
                'concepts': []
            }
        except Exception as e:
            return {
                'success': False,
                'error': str(e),
                'result': None,
                'concepts': []
            }

