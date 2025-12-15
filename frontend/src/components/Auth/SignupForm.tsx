/**
 * Signup form component for Better Auth integration
 */
import React, { useState, ChangeEvent, FormEvent } from 'react';
import authService from '@site/src/services/auth';
import { validateSignupForm, SignupFormData } from './validation';

interface SignupFormProps {
  onSignupSuccess?: (data: any) => void;
}

const SignupForm: React.FC<SignupFormProps> = ({ onSignupSuccess }) => {
  const [formData, setFormData] = useState<SignupFormData>({
    email: '',
    password: '',
    name: '',
    educationLevel: '',
    programmingExperience: '',
    roboticsBackground: ''
  });

  const [errors, setErrors] = useState<Record<string, string>>({});
  const [loading, setLoading] = useState<boolean>(false);

  const educationLevels = ['High School', 'Undergraduate', 'Graduate', 'Professional'];
  const programmingExperiences = ['No Experience', 'Beginner', 'Intermediate', 'Advanced'];
  const roboticsBackgrounds = ['No Experience', 'Hobbyist', 'Academic', 'Professional'];

  const handleChange = (e: ChangeEvent<HTMLInputElement | HTMLSelectElement | HTMLTextAreaElement>) => {
    const { name, value } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: value
    }));

    // Clear error when user starts typing
    if (errors[name]) {
      setErrors(prev => ({
        ...prev,
        [name]: ''
      }));
    }
  };

  const handleSubmit = async (e: FormEvent<HTMLFormElement>) => {
    e.preventDefault();

    const validation = validateSignupForm(formData);
    if (!validation.isValid) {
      setErrors(validation.errors);
      return;
    }

    setLoading(true);
    setErrors({});

    try {
      const response = await authService.signup({
        email: formData.email,
        password: formData.password,
        name: formData.name,
        educationLevel: formData.educationLevel || undefined,
        programmingExperience: formData.programmingExperience || undefined,
        roboticsBackground: formData.roboticsBackground || undefined
      });

      // Call success callback
      if (onSignupSuccess) {
        onSignupSuccess(response);
      }

      // Reset form
      setFormData({
        email: '',
        password: '',
        name: '',
        educationLevel: '',
        programmingExperience: '',
        roboticsBackground: ''
      });
    } catch (error: any) {
      console.error('Signup error:', error);

      if (error.response) {
        // Server responded with error status
        if (error.response.status === 400) {
          setErrors({ general: error.response.data.error || 'Invalid input data' });
        } else if (error.response.status === 409) {
          setErrors({ email: 'User with this email already exists' });
        } else {
          setErrors({ general: 'An error occurred during signup. Please try again.' });
        }
      } else if (error.message) {
        setErrors({ general: error.message });
      } else {
        // Network or other error
        setErrors({ general: 'Network error. Please check your connection and try again.' });
      }
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="signup-form-container">
      <h2>Create Account</h2>

      {errors.general && (
        <div className="error-message" style={{ color: 'red', marginBottom: '10px' }}>
          {errors.general}
        </div>
      )}

      <form onSubmit={handleSubmit} className="signup-form">
        <div className="form-group">
          <label htmlFor="name">Full Name *</label>
          <input
            type="text"
            id="name"
            name="name"
            value={formData.name}
            onChange={handleChange}
            className={errors.name ? 'error' : ''}
            placeholder="Enter your full name"
          />
          {errors.name && <div className="error">{errors.name}</div>}
        </div>

        <div className="form-group">
          <label htmlFor="email">Email *</label>
          <input
            type="email"
            id="email"
            name="email"
            value={formData.email}
            onChange={handleChange}
            className={errors.email ? 'error' : ''}
            placeholder="Enter your email"
          />
          {errors.email && <div className="error">{errors.email}</div>}
        </div>

        <div className="form-group">
          <label htmlFor="password">Password *</label>
          <input
            type="password"
            id="password"
            name="password"
            value={formData.password}
            onChange={handleChange}
            className={errors.password ? 'error' : ''}
            placeholder="Enter your password (min 8 characters)"
          />
          {errors.password && <div className="error">{errors.password}</div>}
        </div>

        <div className="form-group">
          <label htmlFor="educationLevel">Education Level</label>
          <select
            id="educationLevel"
            name="educationLevel"
            value={formData.educationLevel}
            onChange={handleChange}
          >
            <option value="">Select your education level</option>
            {educationLevels.map(level => (
              <option key={level} value={level}>{level}</option>
            ))}
          </select>
        </div>

        <div className="form-group">
          <label htmlFor="programmingExperience">Programming Experience</label>
          <select
            id="programmingExperience"
            name="programmingExperience"
            value={formData.programmingExperience}
            onChange={handleChange}
          >
            <option value="">Select your programming experience</option>
            {programmingExperiences.map(exp => (
              <option key={exp} value={exp}>{exp}</option>
            ))}
          </select>
        </div>

        <div className="form-group">
          <label htmlFor="roboticsBackground">Robotics Background</label>
          <select
            id="roboticsBackground"
            name="roboticsBackground"
            value={formData.roboticsBackground}
            onChange={handleChange}
          >
            <option value="">Select your robotics background</option>
            {roboticsBackgrounds.map(bg => (
              <option key={bg} value={bg}>{bg}</option>
            ))}
          </select>
        </div>

        <button
          type="submit"
          disabled={loading}
          className="signup-button"
          style={{
            padding: '10px 20px',
            backgroundColor: loading ? '#ccc' : '#007cba',
            color: 'white',
            border: 'none',
            borderRadius: '4px',
            cursor: loading ? 'not-allowed' : 'pointer'
          }}
        >
          {loading ? 'Creating Account...' : 'Sign Up'}
        </button>
      </form>
    </div>
  );
};

export default SignupForm;