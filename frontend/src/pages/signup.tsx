import React from 'react';
import Layout from '@theme/Layout';
import SignupForm from '@site/src/components/Auth/SignupForm';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from '@site/src/components/Auth/SignupForm.module.css';

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
            <main className={styles.container}>
                <div className={styles.card}>
                    <SignupForm onSignupSuccess={handleSignupSuccess} />
                </div>
            </main>
        </Layout>
    );
}
