"""
Prompt personalization service for Better Auth + Neon integration
This service adapts system prompts based on user learning preferences
"""
from typing import Dict, Any, Optional
from datetime import datetime

class PromptPersonalizationService:
    """
    Service to personalize AI responses based on user's education level and experience
    """

    def __init__(self):
        # Define mapping of education levels to response complexity
        self.education_complexity_map = {
            "High School": "simple",
            "Undergraduate": "moderate",
            "Graduate": "advanced",
            "Professional": "expert"
        }

        # Define mapping of programming experience to technical depth
        self.experience_depth_map = {
            "No Experience": "basic",
            "Beginner": "introductory",
            "Intermediate": "detailed",
            "Advanced": "comprehensive"
        }

    def get_personalized_system_prompt(self, user_profile: Dict[str, Any], base_prompt: str = "") -> str:
        """
        Generate a personalized system prompt based on user's learning preferences

        Args:
            user_profile: Dictionary containing user's learning preferences
            base_prompt: Base system prompt to enhance

        Returns:
            Personalized system prompt string
        """
        # Start with base prompt or default
        system_prompt = base_prompt if base_prompt else self._get_default_base_prompt()

        # Add personalization based on user level
        education_level = user_profile.get("educationLevel")
        programming_experience = user_profile.get("programmingExperience")
        robotics_background = user_profile.get("roboticsBackground")
        software_background = user_profile.get("softwareBackground", "")
        hardware_background = user_profile.get("hardwareBackground", "")

        # Customize response complexity based on education level
        if education_level:
            complexity_instruction = self._get_complexity_instruction(education_level)
            system_prompt += f"\n\nIMPORTANT: The user is at {education_level} level. {complexity_instruction}"

        # Customize technical depth based on programming experience
        if programming_experience:
            depth_instruction = self._get_depth_instruction(programming_experience)
            system_prompt += f"\n\nIMPORTANT: The user has {programming_experience} programming experience. {depth_instruction}"

        # Add robotics background consideration if applicable
        if robotics_background:
            system_prompt += f"\n\nIMPORTANT: The user has {robotics_background} robotics background. Adapt explanations accordingly."

        # Add background context if provided
        if software_background:
            system_prompt += f"\n\nSOFTWARE BACKGROUND: {software_background}"

        if hardware_background:
            system_prompt += f"\n\nHARDWARE BACKGROUND: {hardware_background}"

        # Add instruction to adjust responses appropriately
        system_prompt += "\n\nAdjust your responses based on the user's education level and experience to ensure appropriate complexity."

        return system_prompt

    def _get_default_base_prompt(self) -> str:
        """
        Get the default base system prompt
        """
        return """You are a helpful AI assistant for a robotics and programming book. Answer questions based on the provided documentation and context."""

    def _get_complexity_instruction(self, education_level: str) -> str:
        """
        Get complexity instruction based on education level
        """
        if education_level == "High School":
            return "Use simple language, analogies, and step-by-step explanations. Avoid jargon and provide foundational context."
        elif education_level == "Undergraduate":
            return "Use standard academic language with some technical terminology. Provide clear explanations with examples."
        elif education_level == "Graduate":
            return "Use advanced technical terminology and assume deeper understanding. Focus on nuanced concepts and applications."
        elif education_level == "Professional":
            return "Use industry-standard terminology and focus on practical applications. Assume comprehensive understanding of fundamentals."
        else:
            return "Use clear language with appropriate explanations for the user's level."

    def _get_depth_instruction(self, programming_experience: str) -> str:
        """
        Get depth instruction based on programming experience
        """
        if programming_experience == "No Experience":
            return "Explain concepts from scratch, using non-technical analogies. Avoid code examples initially."
        elif programming_experience == "Beginner":
            return "Provide gentle introductions to programming concepts with simple code examples."
        elif programming_experience == "Intermediate":
            return "Include detailed code examples and assume basic programming knowledge."
        elif programming_experience == "Advanced":
            return "Focus on complex implementations, best practices, and optimization strategies."
        else:
            return "Provide explanations appropriate for the user's experience level."

    def enhance_prompt_with_context(self, system_prompt: str, context_chunks: list, user_query: str) -> str:
        """
        Enhance the system prompt with context chunks and user query

        Args:
            system_prompt: The personalized system prompt
            context_chunks: List of context chunks from documentation
            user_query: The user's query

        Returns:
            Enhanced prompt with context
        """
        # Build context string
        context_str = "\n\n".join([chunk.get("text", "") for chunk in context_chunks if chunk.get("text")])

        # Enhance the prompt with context
        enhanced_prompt = f"""{system_prompt}

Use the following context to answer the user's question:
{context_str}

User's Question: {user_query}

Provide a helpful, accurate response that addresses the user's question using the context provided and considering their learning preferences."""

        return enhanced_prompt

# Global instance
prompt_personalization_service = PromptPersonalizationService()