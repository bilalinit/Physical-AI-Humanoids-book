import React from 'react';
import Layout from '@theme/Layout';
import SigninForm from '@site/src/components/Auth/SigninForm';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from '@site/src/components/Auth/SigninForm.module.css';

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
            <main className={styles.container}>
                <div className={styles.card}>
                    <SigninForm onSigninSuccess={handleSigninSuccess} />
                </div>
            </main>
        </Layout>
    );
}
