/**
 * Signup form component for Better Auth integration
 */
import React, { useState, ChangeEvent, FormEvent } from 'react';
import authService from '@site/src/services/auth';
import { validateSignupForm, SignupFormData } from './validation';
import styles from './SignupForm.module.css';
import Link from '@docusaurus/Link';

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
    <div className={styles.container}>
      <div className={styles.card}>
        <h2 className={styles.title}>Create Account</h2>

        {errors.general && (
          <div className={styles.generalError}>
            {errors.general}
          </div>
        )}

        <form onSubmit={handleSubmit}>
          <div className={styles.formGroup}>
            <label htmlFor="name" className={styles.label}>Full Name *</label>
            <input
              type="text"
              id="name"
              name="name"
              value={formData.name}
              onChange={handleChange}
              className={`${styles.input} ${errors.name ? styles.inputError : ''}`}
              placeholder="Enter your full name"
            />
            {errors.name && <div className={styles.errorMessage}>{errors.name}</div>}
          </div>

          <div className={styles.formGroup}>
            <label htmlFor="email" className={styles.label}>Email *</label>
            <input
              type="email"
              id="email"
              name="email"
              value={formData.email}
              onChange={handleChange}
              className={`${styles.input} ${errors.email ? styles.inputError : ''}`}
              placeholder="Enter your email"
            />
            {errors.email && <div className={styles.errorMessage}>{errors.email}</div>}
          </div>

          <div className={styles.formGroup}>
            <label htmlFor="password" className={styles.label}>Password *</label>
            <input
              type="password"
              id="password"
              name="password"
              value={formData.password}
              onChange={handleChange}
              className={`${styles.input} ${errors.password ? styles.inputError : ''}`}
              placeholder="Enter your password (min 8 characters)"
            />
            {errors.password && <div className={styles.errorMessage}>{errors.password}</div>}
          </div>

          <div className={styles.formGroup}>
            <label htmlFor="educationLevel" className={styles.label}>Education Level</label>
            <select
              id="educationLevel"
              name="educationLevel"
              value={formData.educationLevel}
              onChange={handleChange}
              className={styles.select}
            >
              <option value="">Select your education level</option>
              {educationLevels.map(level => (
                <option key={level} value={level}>{level}</option>
              ))}
            </select>
          </div>

          <div className={styles.formGroup}>
            <label htmlFor="programmingExperience" className={styles.label}>Programming Experience</label>
            <select
              id="programmingExperience"
              name="programmingExperience"
              value={formData.programmingExperience}
              onChange={handleChange}
              className={styles.select}
            >
              <option value="">Select your programming experience</option>
              {programmingExperiences.map(exp => (
                <option key={exp} value={exp}>{exp}</option>
              ))}
            </select>
          </div>

          <div className={styles.formGroup}>
            <label htmlFor="roboticsBackground" className={styles.label}>Robotics Background</label>
            <select
              id="roboticsBackground"
              name="roboticsBackground"
              value={formData.roboticsBackground}
              onChange={handleChange}
              className={styles.select}
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
            className={`${styles.button} ${loading ? styles.buttonLoading : ''}`}
          >
            {loading && (
              <span className={styles.spinner}></span>
            )}
            {loading ? 'Creating Account...' : 'Sign Up'}
          </button>

          <Link to="/signin" className={styles.navLink}>
            Already have an account? Sign In
          </Link>
        </form>
      </div>
    </div>
  );
};

export default SignupForm;