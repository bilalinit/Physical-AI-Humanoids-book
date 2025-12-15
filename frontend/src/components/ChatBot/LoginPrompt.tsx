import React from 'react';

interface LoginPromptProps {
    colorMode: 'light' | 'dark';
}

const LoginPrompt: React.FC<LoginPromptProps> = ({ colorMode }) => {
    const handleSignUp = () => {
        window.location.href = '/signup';
    };

    const handleSignIn = () => {
        window.location.href = '/signin';
    };

    return (
        <div
            style={{
                width: '100%',
                height: '100%',
                display: 'flex',
                flexDirection: 'column',
                alignItems: 'center',
                justifyContent: 'center',
                padding: '40px 24px',
                backgroundColor: colorMode === 'dark' ? 'var(--ifm-background-surface-color)' : 'var(--ifm-color-white)',
                textAlign: 'center',
            }}
        >
            <div style={{ maxWidth: '320px' }}>
                {/* Icon */}
                <div
                    style={{
                        fontSize: '48px',
                        marginBottom: '20px',
                    }}
                >
                    🔒
                </div>

                {/* Heading */}
                <h3
                    style={{
                        fontSize: '20px',
                        fontWeight: 600,
                        marginBottom: '12px',
                        color: colorMode === 'dark' ? 'var(--ifm-font-color-base)' : 'var(--ifm-color-gray-900)',
                    }}
                >
                    Sign in to use AI Chat
                </h3>

                {/* Description */}
                <p
                    style={{
                        fontSize: '14px',
                        lineHeight: '1.5',
                        marginBottom: '24px',
                        color: colorMode === 'dark' ? 'var(--ifm-color-gray-400)' : 'var(--ifm-color-gray-600)',
                    }}
                >
                    Get personalized answers and access your chat history by signing in or creating an account.
                </p>

                {/* Buttons */}
                <div
                    style={{
                        display: 'flex',
                        flexDirection: 'column',
                        gap: '12px',
                    }}
                >
                    <button
                        onClick={handleSignUp}
                        style={{
                            padding: '12px 24px',
                            backgroundColor: '#4361ee',
                            color: 'white',
                            border: 'none',
                            borderRadius: '8px',
                            fontSize: '14px',
                            fontWeight: 600,
                            cursor: 'pointer',
                            transition: 'background-color 0.2s',
                        }}
                        onMouseEnter={(e) => {
                            e.currentTarget.style.backgroundColor = '#3651de';
                        }}
                        onMouseLeave={(e) => {
                            e.currentTarget.style.backgroundColor = '#4361ee';
                        }}
                    >
                        Create Account
                    </button>

                    <button
                        onClick={handleSignIn}
                        style={{
                            padding: '12px 24px',
                            backgroundColor: 'transparent',
                            color: colorMode === 'dark' ? 'var(--ifm-font-color-base)' : 'var(--ifm-color-gray-900)',
                            border: `1px solid ${colorMode === 'dark' ? 'var(--ifm-color-emphasis-300)' : 'var(--ifm-color-gray-300)'}`,
                            borderRadius: '8px',
                            fontSize: '14px',
                            fontWeight: 600,
                            cursor: 'pointer',
                            transition: 'all 0.2s',
                        }}
                        onMouseEnter={(e) => {
                            e.currentTarget.style.backgroundColor = colorMode === 'dark' ? 'var(--ifm-color-emphasis-100)' : 'var(--ifm-color-gray-100)';
                        }}
                        onMouseLeave={(e) => {
                            e.currentTarget.style.backgroundColor = 'transparent';
                        }}
                    >
                        Sign In
                    </button>
                </div>
            </div>
        </div>
    );
};

export default LoginPrompt;
