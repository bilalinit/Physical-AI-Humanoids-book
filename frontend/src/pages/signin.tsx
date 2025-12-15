import React from 'react';
import Layout from '@theme/Layout';
import SigninForm from '@site/src/components/Auth/SigninForm';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

export default function SigninPage() {
    const { siteConfig } = useDocusaurusContext();

    const handleSigninSuccess = () => {
        // Redirect to homepage after successful signin
        window.location.href = '/';
    };

    return (
        <Layout
            title="Sign In"
            description="Sign in to access your AI chat"
        >
            <main style={{
                minHeight: 'calc(100vh - 60px)',
                display: 'flex',
                alignItems: 'center',
                justifyContent: 'center',
                padding: '2rem'
            }}>
                <div style={{
                    maxWidth: '500px',
                    width: '100%'
                }}>
                    <SigninForm onSigninSuccess={handleSigninSuccess} />

                    {/* Logout button for testing */}
                    <div style={{ marginTop: '20px', textAlign: 'center' }}>
                        <button
                            onClick={() => {
                                localStorage.removeItem('auth_session');
                                localStorage.removeItem('chatkit-thread-id');
                                alert('Logged out! You can now test sign-in.');
                                window.location.reload();
                            }}
                            style={{
                                padding: '8px 16px',
                                backgroundColor: '#dc3545',
                                color: 'white',
                                border: 'none',
                                borderRadius: '4px',
                                cursor: 'pointer'
                            }}
                        >
                            Logout (Clear Session)
                        </button>
                    </div>
                </div>
            </main>
        </Layout>
    );
}
