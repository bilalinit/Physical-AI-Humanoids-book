import React from 'react';
import Layout from '@theme/Layout';
import SignupForm from '@site/src/components/Auth/SignupForm';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

export default function SignupPage() {
    const { siteConfig } = useDocusaurusContext();

    const handleSignupSuccess = () => {
        // Redirect to homepage after successful signup
        window.location.href = '/';
    };

    return (
        <Layout
            title="Sign Up"
            description="Create an account to access personalized AI chat"
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
                    <SignupForm onSignupSuccess={handleSignupSuccess} />
                </div>
            </main>
        </Layout>
    );
}
