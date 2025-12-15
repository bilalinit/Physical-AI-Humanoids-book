/**
 * Validation utilities for auth forms
 */

export interface SignupFormData {
  email?: string;
  password?: string;
  name?: string;
  educationLevel?: string;
  programmingExperience?: string;
  roboticsBackground?: string;
}

export interface SigninFormData {
  email?: string;
  password?: string;
}

export interface ValidationResult {
  isValid: boolean;
  errors: Record<string, string>;
}

// Email validation
export const validateEmail = (email: string): boolean => {
  const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
  return emailRegex.test(email);
};

// Password validation
export const validatePassword = (password: string): boolean => {
  return !!password && password.length >= 8;
};

// Name validation
export const validateName = (name: string): boolean => {
  return !!name && name.trim().length >= 2 && name.trim().length <= 100;
};

// Form validation for signup
export const validateSignupForm = (formData: SignupFormData): ValidationResult => {
  const errors: Record<string, string> = {};

  if (!formData.name) {
    errors.name = 'Name is required';
  } else if (!validateName(formData.name)) {
    errors.name = 'Name must be between 2 and 100 characters';
  }

  if (!formData.email) {
    errors.email = 'Email is required';
  } else if (!validateEmail(formData.email)) {
    errors.email = 'Please enter a valid email address';
  }

  if (!formData.password) {
    errors.password = 'Password is required';
  } else if (!validatePassword(formData.password)) {
    errors.password = 'Password must be at least 8 characters long';
  }

  // Validate education level if provided
  if (formData.educationLevel && !['High School', 'Undergraduate', 'Graduate', 'Professional'].includes(formData.educationLevel)) {
    errors.educationLevel = 'Please select a valid education level';
  }

  // Validate programming experience if provided
  if (formData.programmingExperience && !['No Experience', 'Beginner', 'Intermediate', 'Advanced'].includes(formData.programmingExperience)) {
    errors.programmingExperience = 'Please select a valid programming experience level';
  }

  // Validate robotics background if provided
  if (formData.roboticsBackground && !['No Experience', 'Hobbyist', 'Academic', 'Professional'].includes(formData.roboticsBackground)) {
    errors.roboticsBackground = 'Please select a valid robotics background';
  }

  return {
    isValid: Object.keys(errors).length === 0,
    errors
  };
};

// Form validation for signin
export const validateSigninForm = (formData: SigninFormData): ValidationResult => {
  const errors: Record<string, string> = {};

  if (!formData.email) {
    errors.email = 'Email is required';
  } else if (!validateEmail(formData.email)) {
    errors.email = 'Please enter a valid email address';
  }

  if (!formData.password) {
    errors.password = 'Password is required';
  }

  return {
    isValid: Object.keys(errors).length === 0,
    errors
  };
};

// Validation for profile updates
export const validateProfileUpdate = (profileData: SignupFormData): ValidationResult => {
  const errors: Record<string, string> = {};

  // Validate name if provided
  if (profileData.name && !validateName(profileData.name)) {
    errors.name = 'Name must be between 2 and 100 characters';
  }

  // Validate email if provided
  if (profileData.email && !validateEmail(profileData.email)) {
    errors.email = 'Please enter a valid email address';
  }

  // Validate education level if provided
  if (profileData.educationLevel && !['High School', 'Undergraduate', 'Graduate', 'Professional'].includes(profileData.educationLevel)) {
    errors.educationLevel = 'Please select a valid education level';
  }

  // Validate programming experience if provided
  if (profileData.programmingExperience && !['No Experience', 'Beginner', 'Intermediate', 'Advanced'].includes(profileData.programmingExperience)) {
    errors.programmingExperience = 'Please select a valid programming experience level';
  }

  // Validate robotics background if provided
  if (profileData.roboticsBackground && !['No Experience', 'Hobbyist', 'Academic', 'Professional'].includes(profileData.roboticsBackground)) {
    errors.roboticsBackground = 'Please select a valid robotics background';
  }

  return {
    isValid: Object.keys(errors).length === 0,
    errors
  };
};

// Helper to format error messages
export const formatErrorMessage = (error: any): string => {
  if (error.response && error.response.data && error.response.data.error) {
    return error.response.data.error;
  }
  return error.message || 'An error occurred';
};