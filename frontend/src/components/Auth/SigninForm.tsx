/**
 * Signin form component for Better Auth integration
 */
import React, { useState, ChangeEvent, FormEvent } from 'react';
import authService from '@site/src/services/auth';
import { validateSigninForm, SigninFormData } from './validation';
import styles from './SigninForm.module.css';
import Link from '@docusaurus/Link';

interface SigninFormProps {
  onSigninSuccess?: (data: any) => void;
}

const SigninForm: React.FC<SigninFormProps> = ({ onSigninSuccess }) => {
  const [formData, setFormData] = useState<SigninFormData>({
    email: '',
    password: ''
  });

  const [errors, setErrors] = useState<Record<string, string>>({});
  const [loading, setLoading] = useState<boolean>(false);

  const handleChange = (e: ChangeEvent<HTMLInputElement>) => {
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

    const validation = validateSigninForm(formData);
    if (!validation.isValid) {
      setErrors(validation.errors);
      return;
    }

    setLoading(true);
    setErrors({});

    try {
      const response = await authService.signin({
        email: formData.email,
        password: formData.password
      });

      // Store session in localStorage for chatbot to access
      if (response.session) {
        localStorage.setItem('auth_session', JSON.stringify(response.session));
      }

      // Call success callback
      if (onSigninSuccess) {
        onSigninSuccess(response);
      }

      // Reset form
      setFormData({
        email: '',
        password: ''
      });
    } catch (error: any) {
      console.error('Signin error:', error);

      if (error.body?.message) {
        setErrors({ general: error.body.message });
      } else if (error.response) {
        // Server responded with error status
        if (error.response.status === 400) {
          setErrors({ general: error.response.data?.error || error.response.data?.message || 'Invalid input data' });
        } else if (error.response.status === 401) {
          setErrors({ general: error.response.data?.message || 'Invalid email or password' });
        } else if (error.response.status === 404) {
          setErrors({ general: error.response.data?.message || 'User not found' });
        } else {
          setErrors({ general: error.response.data?.message || error.response.data?.error || 'An error occurred during signin. Please try again.' });
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
        <h2 className={styles.title}>Sign In</h2>

        {errors.general && (
          <div className={styles.generalError}>
            {errors.general}
          </div>
        )}

        <form onSubmit={handleSubmit}>
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
              placeholder="Enter your password"
            />
            {errors.password && <div className={styles.errorMessage}>{errors.password}</div>}
          </div>

          <button
            type="submit"
            disabled={loading}
            className={`${styles.button} ${loading ? styles.buttonLoading : ''}`}
          >
            {loading && (
              <span className={styles.spinner}></span>
            )}
            {loading ? 'Signing In...' : 'Sign In'}
          </button>

          <Link to="/signup" className={styles.navLink}>
            Don't have an account? Sign Up
          </Link>
        </form>
      </div>
    </div>
  );
};

export default SigninForm;