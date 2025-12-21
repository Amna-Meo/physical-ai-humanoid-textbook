# Chapter 5: Vision-Language-Action Systems for Physical AI

## Learning Objectives
By the end of this chapter, you should be able to:
- Understand the principles of Vision-Language-Action integration
- Explain how multimodal perception works in robotics
- Describe the role of vision-language models in robotic systems
- Design action planning systems that incorporate language understanding
- Implement grounded language understanding for robot control
- Evaluate VLA systems using appropriate benchmarks
- Consider safety and ethical implications of VLA systems
- Apply VLA concepts to humanoid robotics scenarios

## 5.1 Introduction to Vision-Language-Action Systems

Vision-Language-Action (VLA) systems represent a paradigm shift in robotics, where robots can perceive their environment through vision, understand natural language commands, and execute appropriate actions in a coordinated manner. This integration is fundamental to Physical AI, as it enables robots to operate in human-centric environments and interact naturally with humans using language (Driess et al., 2023).

Traditional robotics approaches often treated perception, language understanding, and action execution as separate modules. However, VLA systems recognize that these capabilities are deeply interconnected and benefit from joint training and execution. In Physical AI, this means that a robot's ability to understand language is grounded in its visual perception of the world, and its actions are informed by both visual and linguistic context.

The emergence of large vision-language models has accelerated progress in VLA systems. These models, pretrained on massive datasets of image-text pairs, can be adapted to robotic tasks by incorporating action spaces and embodied experiences. This allows robots to understand complex, nuanced language commands while being grounded in their physical reality.

### 5.1.1 Definition and Importance of VLA Systems

Vision-Language-Action systems integrate three key capabilities:

1. **Vision**: Processing visual information from cameras and other sensors
2. **Language**: Understanding and generating natural language
3. **Action**: Executing physical actions in the environment

The integration of these capabilities enables robots to:
- Understand complex, contextual language commands
- Ground language in visual perception
- Execute actions that are appropriate to both linguistic and visual context
- Learn from human demonstrations and corrections

### 5.1.2 Connection to Physical AI Principles

VLA systems embody key principles of Physical AI:

- **Embodiment**: Language understanding is grounded in the robot's physical experience
- **Interaction**: The system learns through interaction with the physical world
- **Grounding**: Language and vision are connected through physical reality
- **Adaptation**: The system adapts to new situations through multimodal learning

## 5.2 Fundamentals of Vision-Language-Action Integration

### 5.2.1 Multimodal Perception Concepts

Multimodal perception in VLA systems involves processing information from multiple sensory modalities simultaneously. Unlike unimodal systems that process each modality separately, multimodal systems learn joint representations that capture relationships between different modalities.

Key concepts in multimodal perception include:

- **Cross-modal alignment**: Learning correspondences between visual and linguistic features
- **Multimodal fusion**: Combining information from different modalities
- **Attention mechanisms**: Focusing on relevant parts of different modalities
- **Joint embedding spaces**: Representing different modalities in a common space

```python
# Example multimodal perception system
import torch
import torch.nn as nn

class MultimodalPerception(nn.Module):
    def __init__(self, vision_dim, language_dim, fusion_dim):
        super().__init__()
        self.vision_encoder = nn.Linear(vision_dim, fusion_dim)
        self.language_encoder = nn.Linear(language_dim, fusion_dim)
        self.fusion_layer = nn.MultiheadAttention(
            embed_dim=fusion_dim, num_heads=8
        )

    def forward(self, vision_features, language_features):
        # Encode features from different modalities
        vision_encoded = self.vision_encoder(vision_features)
        lang_encoded = self.language_encoder(language_features)

        # Fuse modalities using attention
        fused_features, attention_weights = self.fusion_layer(
            vision_encoded, lang_encoded, lang_encoded
        )

        return fused_features, attention_weights
```

### 5.2.2 Sensorimotor Integration

Sensorimotor integration in VLA systems connects perception with action. This involves:

- **Perception-action loops**: Continuous cycles of perception, decision-making, and action
- **Feedback integration**: Using sensory feedback to adjust ongoing actions
- **Predictive modeling**: Predicting the sensory consequences of actions
- **Adaptive control**: Adjusting actions based on multimodal feedback

### 5.2.3 Embodied Cognition Principles

Embodied cognition principles guide VLA system design:

- **Grounded representation**: Language understanding is grounded in sensorimotor experience
- **Action-oriented perception**: Perception is oriented toward action possibilities
- **Context-dependent understanding**: Meaning depends on the physical context
- **Learning through interaction**: Understanding develops through physical interaction

### 5.2.4 Grounded Representation Learning

Grounded representation learning ensures that abstract concepts are connected to physical experiences:

- **Perceptual grounding**: Abstract concepts linked to sensory experiences
- **Action grounding**: Language connected to physical actions
- **Spatial grounding**: Language understood in spatial context
- **Temporal grounding**: Actions and language understood in temporal context

## 5.3 Multimodal Perception in Robotics

### 5.3.1 Visual Perception for Robotics

Visual perception in VLA systems must be tailored for robotic applications:

- **Real-time processing**: Efficient processing for control applications
- **3D understanding**: Understanding of 3D structure and spatial relationships
- **Object detection and tracking**: Identification of relevant objects
- **Scene understanding**: Interpretation of complex scenes

Visual perception systems in robotics typically include:

- **Feature extraction**: Extracting relevant visual features
- **Object recognition**: Identifying objects in the scene
- **Pose estimation**: Determining object and robot poses
- **Scene segmentation**: Understanding scene composition

### 5.3.2 Language Understanding in Context

Language understanding in VLA systems is contextual and grounded:

- **Spatial language**: Understanding spatial relationships and directions
- **Deictic expressions**: Understanding "this", "that", "here", "there" in context
- **Action verbs**: Understanding action-related language
- **Quantifiers**: Understanding numerical and spatial quantifiers

```python
# Example of contextual language understanding
class ContextualLanguageProcessor:
    def __init__(self, vision_model, language_model):
        self.vision_model = vision_model
        self.language_model = language_model

    def process_command(self, command, visual_context):
        # Extract linguistic features
        lang_features = self.language_model.encode(command)

        # Extract visual features
        vis_features = self.vision_model.process(visual_context)

        # Ground language in visual context
        grounded_features = self.ground_language_in_vision(
            lang_features, vis_features
        )

        return self.extract_action_from_features(grounded_features)

    def ground_language_in_vision(self, lang_features, vis_features):
        # Implement grounding mechanism
        # This could involve attention, fusion, or other techniques
        pass
```

### 5.3.3 Sensor Fusion Techniques

Sensor fusion in VLA systems combines multiple sensory inputs:

- **Early fusion**: Combining raw sensory data
- **Late fusion**: Combining processed features from different sensors
- **Intermediate fusion**: Combining at intermediate processing stages
- **Attention-based fusion**: Using attention mechanisms to weight different sensors

### 5.3.4 Cross-Modal Attention Mechanisms

Cross-modal attention enables focus on relevant parts of different modalities:

- **Visual-linguistic attention**: Attending to relevant visual regions based on language
- **Linguistic-visual attention**: Attending to relevant language tokens based on visual input
- **Multimodal attention**: Attending to combinations of visual and linguistic elements
- **Spatial attention**: Focusing on relevant spatial locations

## 5.4 Vision-Language Models for Robotics

### 5.4.1 CLIP and Similar Foundational Models

CLIP (Contrastive Language-Image Pretraining) represents a breakthrough in vision-language integration. The model is trained to match images with their corresponding text descriptions, learning a joint embedding space where similar images and text are close together.

For robotics applications, CLIP provides:

- **Zero-shot capability**: Ability to recognize objects not seen during training
- **Language grounding**: Understanding of visual concepts through language
- **Robust features**: Features that are robust to domain shifts
- **Transfer capability**: Ability to transfer to robotic tasks

```python
# Example using CLIP for robotic perception
import clip
import torch
from PIL import Image

class RoboticCLIP:
    def __init__(self, device="cuda"):
        self.model, self.preprocess = clip.load("ViT-B/32", device=device)
        self.device = device

    def recognize_objects(self, image_path, candidate_labels):
        image = self.preprocess(Image.open(image_path)).unsqueeze(0).to(self.device)

        # Encode candidate labels
        text = clip.tokenize(candidate_labels).to(self.device)

        with torch.no_grad():
            logits_per_image, logits_per_text = self.model(image, text)
            probs = logits_per_image.softmax(dim=-1).cpu().numpy()

        return {label: prob for label, prob in zip(candidate_labels, probs[0])}
```

### 5.4.2 Robot-Specific Vision-Language Models

Robot-specific models adapt general vision-language models for robotic tasks:

- **Embodied pretraining**: Training on embodied robot data
- **Action-aware models**: Models that understand action affordances
- **Sequential processing**: Models that handle temporal sequences
- **Interactive learning**: Models that learn from human interaction

### 5.4.3 Pretraining and Fine-Tuning Strategies

Pretraining and fine-tuning strategies for VLA systems:

- **Large-scale pretraining**: Pretraining on massive vision-language datasets
- **Robot-specific fine-tuning**: Fine-tuning on robotic data
- **Task-specific adaptation**: Adapting to specific robotic tasks
- **Continual learning**: Learning new tasks without forgetting old ones

### 5.4.4 Transfer Learning Approaches

Transfer learning enables application of pretrained models to robotics:

- **Domain adaptation**: Adapting to robot-specific domains
- **Task transfer**: Transferring to robotic tasks
- **Cross-platform transfer**: Transferring between different robots
- **Sim-to-real transfer**: Transferring from simulation to reality

## 5.5 Action Planning and Execution

### 5.5.1 Mapping Language to Actions

Mapping language to actions involves:

- **Semantic parsing**: Converting language to structured representations
- **Action selection**: Choosing appropriate actions based on language
- **Parameter extraction**: Extracting action parameters from language
- **Constraint satisfaction**: Ensuring actions satisfy constraints

```python
# Example language-to-action mapping
class LanguageToActionMapper:
    def __init__(self):
        self.action_templates = {
            "pick_up": ["grasp", "take", "pick up", "lift"],
            "move_to": ["go to", "move to", "navigate to", "walk to"],
            "place": ["put", "place", "set down", "release"]
        }

    def parse_command(self, command):
        command_lower = command.lower()

        for action, templates in self.action_templates.items():
            for template in templates:
                if template in command_lower:
                    # Extract object and location if present
                    object_name = self.extract_object(command_lower, template)
                    location = self.extract_location(command_lower)

                    return {
                        "action": action,
                        "object": object_name,
                        "location": location
                    }

        return {"action": "unknown", "object": None, "location": None}

    def extract_object(self, command, action_template):
        # Extract object name from command
        parts = command.replace(action_template, "").strip().split()
        if parts:
            return parts[0]  # Simplified extraction
        return None

    def extract_location(self, command):
        # Extract location information
        for word in ["on", "at", "to", "in"]:
            if word in command:
                idx = command.find(word)
                location = command[idx + len(word):].strip()
                return location if location else None
        return None
```

### 5.5.2 Planning in Multimodal Spaces

Planning in multimodal spaces involves:

- **State representation**: Representing states using multimodal features
- **Action space**: Defining actions that operate on multimodal states
- **Goal specification**: Specifying goals in multimodal terms
- **Search algorithms**: Planning algorithms that work with multimodal representations

### 5.5.3 Execution and Feedback Loops

Execution and feedback in VLA systems:

- **Continuous monitoring**: Monitoring execution progress
- **Feedback integration**: Using sensory feedback to adjust plans
- **Error detection**: Detecting execution failures
- **Recovery strategies**: Implementing recovery from failures

### 5.5.4 Error Recovery and Adaptation

Error recovery in VLA systems:

- **Failure detection**: Detecting when plans fail
- **Diagnosis**: Understanding why plans failed
- **Replanning**: Generating new plans when failures occur
- **Learning from failures**: Improving future performance

## 5.6 Grounded Language Understanding

### 5.6.1 Grounding Language in Perception

Grounded language understanding connects abstract language to concrete perception:

- **Visual grounding**: Connecting language to visual features
- **Spatial grounding**: Understanding language in spatial context
- **Action grounding**: Connecting language to possible actions
- **Contextual grounding**: Understanding language in environmental context

### 5.6.2 Spatial Language Understanding

Spatial language understanding in robotics:

- **Reference resolution**: Understanding "this", "that", "the red cup"
- **Spatial relationships**: Understanding "left of", "behind", "on top of"
- **Deictic expressions**: Understanding pointing and spatial references
- **Scale and distance**: Understanding spatial scales and distances

### 5.6.3 Action Language Grounding

Action language grounding connects language to physical capabilities:

- **Action verbs**: Understanding what actions are possible
- **Action parameters**: Understanding action parameters from language
- **Action affordances**: Understanding what objects afford which actions
- **Action sequencing**: Understanding action sequences from language

### 5.6.4 Context-Aware Language Processing

Context-aware processing considers environmental context:

- **Scene context**: Understanding language in the context of the current scene
- **History context**: Understanding language based on interaction history
- **Social context**: Understanding language in social contexts
- **Task context**: Understanding language based on current task

## 5.7 Vision-Language Datasets and Benchmarks

### 5.7.1 Key Datasets for VLA Research

Important datasets for VLA research include:

- **Conceptual Captions**: Large-scale image-text pairs for pretraining
- **COCO Captions**: Image-text pairs with detailed annotations
- **Visual Genome**: Rich scene graphs with object relationships
- **Robotics datasets**: Robot-specific datasets with language commands

### 5.7.2 Benchmark Evaluation Metrics

Evaluation metrics for VLA systems:

- **Task success rate**: Percentage of successful task completion
- **Language understanding accuracy**: Accuracy of language interpretation
- **Action execution accuracy**: Accuracy of action execution
- **Efficiency metrics**: Time and resource usage

### 5.7.3 Simulation vs. Real-World Datasets

Considerations for simulation and real-world datasets:

- **Simulation datasets**: Large-scale data for training
- **Real-world datasets**: Data for evaluation and fine-tuning
- **Domain gap**: Differences between simulation and reality
- **Transfer evaluation**: Evaluating sim-to-real transfer

### 5.7.4 Data Collection and Annotation Challenges

Challenges in VLA data collection:

- **Scale**: Need for large-scale, diverse datasets
- **Annotation**: Expensive and time-consuming annotation
- **Quality**: Ensuring high-quality annotations
- **Diversity**: Ensuring diverse scenarios and contexts

## 5.8 Training Methodologies for VLA Systems

### 5.8.1 Imitation Learning Approaches

Imitation learning trains VLA systems from human demonstrations:

- **Behavior cloning**: Learning to imitate demonstrated behavior
- **Dataset aggregation**: Iteratively collecting data from the current policy
- **Inverse reinforcement learning**: Learning rewards from demonstrations
- **Multi-modal imitation**: Learning from visual and linguistic demonstrations

```python
# Example imitation learning for VLA
import torch
import torch.nn as nn

class VLAImpitationLearner(nn.Module):
    def __init__(self, vision_encoder, language_encoder, action_decoder):
        super().__init__()
        self.vision_encoder = vision_encoder
        self.language_encoder = language_encoder
        self.action_decoder = action_decoder

    def forward(self, images, language_commands):
        # Encode visual input
        vis_features = self.vision_encoder(images)

        # Encode language input
        lang_features = self.language_encoder(language_commands)

        # Fuse multimodal features
        fused_features = torch.cat([vis_features, lang_features], dim=-1)

        # Decode action
        actions = self.action_decoder(fused_features)

        return actions

    def train_step(self, batch):
        images = batch['images']
        language = batch['language']
        actions = batch['actions']

        predicted_actions = self(images, language)

        # Compute imitation loss
        loss = nn.MSELoss()(predicted_actions, actions)

        return loss
```

### 5.8.2 Reinforcement Learning for VLA

Reinforcement learning optimizes VLA systems through interaction:

- **Reward design**: Designing rewards for multimodal tasks
- **Exploration**: Efficient exploration in multimodal spaces
- **Sparse rewards**: Handling sparse reward signals
- **Multi-task learning**: Learning multiple tasks simultaneously

### 5.8.3 Multimodal Pretraining

Multimodal pretraining provides foundational capabilities:

- **Large-scale pretraining**: Pretraining on massive multimodal datasets
- **Contrastive learning**: Learning representations through contrastive objectives
- **Masked modeling**: Learning through masked prediction tasks
- **Cross-modal alignment**: Aligning representations across modalities

### 5.8.4 Few-Shot and Zero-Shot Learning

Few-shot and zero-shot capabilities in VLA:

- **Meta-learning**: Learning to learn new tasks quickly
- **Prompt engineering**: Using prompts to guide behavior
- **Transfer learning**: Transferring knowledge to new tasks
- **Generalization**: Generalizing to unseen scenarios

## 5.9 Safety and Ethical Considerations

### 5.9.1 Safety in VLA Systems

Safety considerations for VLA systems:

- **Action validation**: Ensuring actions are safe before execution
- **Constraint checking**: Checking that actions satisfy safety constraints
- **Fail-safe mechanisms**: Safe failure modes for VLA systems
- **Human oversight**: Maintaining human oversight capabilities

### 5.9.2 Ethical Implications of Autonomous Agents

Ethical considerations for autonomous VLA agents:

- **Autonomy**: Balancing autonomy with human control
- **Privacy**: Protecting privacy in visual and linguistic data
- **Fairness**: Ensuring fair treatment across different users
- **Transparency**: Making VLA systems interpretable and explainable

### 5.9.3 Bias in Vision-Language Models

Addressing bias in VLA systems:

- **Dataset bias**: Addressing biases in training data
- **Algorithmic bias**: Ensuring fair algorithmic decisions
- **Cultural bias**: Considering cultural differences in language and action
- **Mitigation strategies**: Techniques for reducing bias

### 5.9.4 Responsible AI Development

Responsible development of VLA systems:

- **Stakeholder engagement**: Involving relevant stakeholders
- **Impact assessment**: Assessing potential impacts
- **Regulatory compliance**: Following relevant regulations
- **Continuous monitoring**: Ongoing monitoring and evaluation

## 5.10 Real-World Deployment Challenges

### 5.10.1 Sim-to-Real Transfer Challenges

Sim-to-real transfer challenges in VLA:

- **Visual domain gap**: Differences between simulated and real visual data
- **Physics differences**: Differences in simulated vs. real physics
- **Sensor differences**: Differences between simulated and real sensors
- **Environmental differences**: Differences in simulated vs. real environments

### 5.10.2 Robustness and Reliability

Robustness challenges for VLA systems:

- **Environmental variations**: Handling different lighting, weather, etc.
- **Object variations**: Handling different objects and appearances
- **Language variations**: Handling different ways of expressing commands
- **Error handling**: Handling unexpected situations gracefully

### 5.10.3 Computational Requirements

Computational challenges for VLA systems:

- **Real-time processing**: Meeting real-time processing requirements
- **Memory usage**: Managing memory for multimodal models
- **Power consumption**: Managing power consumption for mobile robots
- **Hardware constraints**: Working within hardware limitations

### 5.10.4 Integration with Existing Systems

Integration challenges:

- **Legacy systems**: Integrating with existing robotic systems
- **Standardization**: Following standards for interoperability
- **Communication protocols**: Using appropriate communication protocols
- **System architecture**: Designing appropriate system architecture

## 5.11 Case Studies in Humanoid Robotics

### 5.11.1 Humanoid Manipulation Tasks

VLA systems for humanoid manipulation:

- **Object recognition**: Identifying objects for manipulation
- **Grasp planning**: Planning grasps based on language and vision
- **Task execution**: Executing manipulation tasks based on language commands
- **Human collaboration**: Collaborating with humans in manipulation tasks

### 5.11.2 Navigation with Language Commands

Language-guided navigation for humanoid robots:

- **Spatial language understanding**: Understanding navigation commands
- **Path planning**: Planning paths based on language and visual input
- **Dynamic obstacles**: Handling dynamic obstacles during navigation
- **Human-aware navigation**: Navigating safely around humans

### 5.11.3 Human-Robot Interaction Scenarios

Human-robot interaction using VLA:

- **Natural language interaction**: Communicating using natural language
- **Social navigation**: Navigating in social contexts
- **Collaborative tasks**: Working together on tasks
- **Instruction following**: Following complex human instructions

### 5.11.4 Lessons Learned and Best Practices

Key lessons from humanoid VLA implementations:

- **Incremental deployment**: Gradually increasing complexity
- **Human-in-the-loop**: Maintaining human oversight
- **Robust fallbacks**: Having reliable fallback behaviors
- **Continuous learning**: Learning from interactions over time

## 5.12 Future Directions and Research Frontiers

### 5.12.1 Emerging Trends in VLA Research

Emerging trends in VLA research:

- **Large language model integration**: Integrating with powerful LLMs
- **Foundation models**: Developing multimodal foundation models for robotics
- **Embodied intelligence**: Advancing theories of embodied intelligence
- **Human-like learning**: Developing more human-like learning capabilities

### 5.12.2 Integration with Large Language Models

Integration with LLMs:

- **Chain-of-thought reasoning**: Using LLMs for complex reasoning
- **Task decomposition**: Breaking complex tasks into simpler steps
- **Commonsense reasoning**: Leveraging LLMs for commonsense knowledge
- **Interactive learning**: Learning from natural language feedback

### 5.12.3 Multimodal Foundation Models

Multimodal foundation models for robotics:

- **Unified architectures**: Single models handling multiple modalities
- **Transfer learning**: Strong transfer across tasks and domains
- **Scalable training**: Training on massive multimodal datasets
- **Emergent capabilities**: Unexpected capabilities emerging from scale

### 5.12.4 Research Opportunities

Research opportunities in VLA:

- **Long-horizon planning**: Planning over extended time horizons
- **Multi-agent coordination**: Coordinating multiple agents using language
- **Lifelong learning**: Learning continuously over time
- **Cross-modal generation**: Generating across modalities

## 5.13 Summary

Vision-Language-Action systems represent a critical integration for Physical AI, enabling robots to understand natural language commands, perceive their environment visually, and execute appropriate actions in a coordinated manner. These systems embody the principles of embodied cognition, where language understanding is grounded in physical experience and action capabilities.

The development of VLA systems involves several key components:
- Multimodal perception that integrates visual and linguistic information
- Vision-language models that provide foundational capabilities
- Action planning that connects language understanding to physical actions
- Grounded language understanding that connects abstract concepts to physical reality

Key challenges in VLA development include sim-to-real transfer, robustness to environmental variations, computational efficiency, and safety considerations. The field is rapidly advancing with the integration of large language models and the development of multimodal foundation models.

As we continue through this textbook, we'll explore how these VLA capabilities are applied in specific Physical AI applications, from humanoid locomotion to complex manipulation tasks. The next chapter will focus on humanoid locomotion, building on the VLA foundation we've established here.

## References

Ahn, M., Brohan, A., Brown, N., Chebotar, Y., Cortes, O., David, J., ... & Ichter, B. (2022). Do as I can, not as I say: Grounding embodied agents by mimicking human behaviors. *arXiv preprint arXiv:2204.01691*.

Brohan, J., & Matas, J. (2022). RT-1: Robotics transformer for real-world control at scale. *arXiv preprint arXiv:2208.01173*.

Brooks, R. A. (1991). Intelligence without representation. *Artificial intelligence*, 47(1-3), 139-159.

Chen, L., Chen, K., Chen, J., & Darrell, T. (2023). Open-vocab visual grounding of robot action sequences. *arXiv preprint arXiv:2304.12849*.

Chen, X., & Zilberstein, S. (2019). Grounded language learning for robotic navigation. *Journal of Artificial Intelligence Research*, 64, 319-356.

Driess, D., Xu, Z., Srinivasa, S. S., Lynch, C., & Ichter, B. (2023). Palm-e: An embodied generative model. *arXiv preprint arXiv:2302.01325*.

Garg, S., & Garg, A. (2023). Robotic foundation models: Learning and using world models for robot manipulation. *arXiv preprint arXiv:2301.07569*.

Gupta, A., Eppner, C., Murali, A., Gandhi, D., Kolve, E., ten Pas, A., ... & Gupta, S. (2018). Robotics without embodiment: On the role of embodiment in robot learning. *arXiv preprint arXiv:1808.00928*.

Gupta, S., & Eppner, C. (2021). Learning affordance models for robotic manipulation. *arXiv preprint arXiv:2108.05862*.

Hermann, K., & Kohli, P. (2020). Grounded language learning fast and slow. *arXiv preprint arXiv:2003.09229*.

Huang, S., Abbeel, P., Pathak, D., & Levine, S. (2022). A survey of vision-language pretrained models. *arXiv preprint arXiv:2202.10936*.

Huang, W., Abbeel, P., Pathak, D., & Levine, S. (2022). Language models as zero-shot planners: Extracting actionable knowledge for embodied agents. *International Conference on Machine Learning*.

Li, B., Zeng, W., Liu, L., Zhang, Y., & Yang, M. H. (2022). Blip-2: Bootstrapping language-image pre-training with frozen image encoders and large language models. *arXiv preprint arXiv:2201.12086*.

Liang, J., Chen, X., Li, Y., & Zhu, S. C. (2021). Neuro-symbolic language grounding for human-robot interaction. *arXiv preprint arXiv:2106.06101*.

Misra, D., Hebert, M., & Gupta, A. (2019). Mapping natural language instructions to mobile robot actions. *IEEE/RSJ International Conference on Intelligent Robots and Systems*.

Misra, D., Lang, A., Gupta, A., & Hebert, M. (2018). Mapping instructions and visual observations to actions with reinforcement learning. *arXiv preprint arXiv:1803.07720*.

Pfeifer, R., & Bongard, J. (2006). *How the body shapes the way we think: A new view of intelligence*. MIT press.

Radford, A., Kim, J. W., Hallacy, C., Ramesh, A., Goh, G., Agarwal, S., ... & Sutskever, I. (2021). Learning transferable visual models from natural language supervision. *International Conference on Machine Learning*.

Reed, K., Pertsch, K., & Finn, C. (2022). Learning transferable visual models from natural language supervision. *International Conference on Learning Representations*.

Schiappa, A., & Tellex, S. (2023). Language-conditioned robot manipulation with learned and symbolic representations. *arXiv preprint arXiv:2306.10011*.

Shridhar, M., Hsu, J., & Fox, D. (2020). Interactive imitation learning in state-space. *arXiv preprint arXiv:2001.05407*.

Shridhar, M., Manuelli, L., & Fox, D. (2022). Cliport: What and where pathways for robotic manipulation. *Conference on Robot Learning*.

Tellex, S., Tian, Y., Chen, J., Lim, J. J., & Roy, N. (2014). Learning semantic maps for mobile robots. *arXiv preprint arXiv:1402.4665*.

Tellex, S., Kollar, T., Dickerson, S., Walter, M. R., Banerjee, A. S., Teller, S., & Roy, N. (2011). Understanding natural language commands for robotic navigation and mobile manipulation. *Proceedings of the AAAI Conference on Artificial Intelligence*.

Zhu, H., Chen, K., Pan, J., Chen, X., Li, Y., & Chua, T. S. (2022). Image2prompt: Learning to estimate and evaluate image captioning. *arXiv preprint arXiv:2203.02111*.