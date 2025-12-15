/**
 * Signin form component for Better Auth integration
 */
import React, { useState, ChangeEvent, FormEvent } from 'react';
import authService from '@site/src/services/auth';
import { validateSigninForm, SigninFormData } from './validation';

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

      if (error.response) {
        // Server responded with error status
        if (error.response.status === 400) {
          setErrors({ general: error.response.data.error || 'Invalid input data' });
        } else if (error.response.status === 401) {
          setErrors({ general: 'Invalid email or password' });
        } else {
          setErrors({ general: 'An error occurred during signin. Please try again.' });
        }
      } else {
        // Network or other error
        setErrors({ general: 'Network error. Please check your connection and try again.' });
      }
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="signin-form-container">
      <h2>Sign In</h2>

      {errors.general && (
        <div className="error-message" style={{ color: 'red', marginBottom: '10px' }}>
          {errors.general}
        </div>
      )}

      <form onSubmit={handleSubmit} className="signin-form">
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
            placeholder="Enter your password"
          />
          {errors.password && <div className="error">{errors.password}</div>}
        </div>

        <button
          type="submit"
          disabled={loading}
          className="signin-button"
          style={{
            padding: '10px 20px',
            backgroundColor: loading ? '#ccc' : '#007cba',
            color: 'white',
            border: 'none',
            borderRadius: '4px',
            cursor: loading ? 'not-allowed' : 'pointer'
          }}
        >
          {loading ? 'Signing In...' : 'Sign In'}
        </button>
      </form>
    </div>
  );
};

export default SigninForm;