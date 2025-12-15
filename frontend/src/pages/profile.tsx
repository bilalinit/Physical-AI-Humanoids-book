import React from 'react';
import Layout from '@theme/Layout';
import Profile from '@site/src/components/Auth/Profile';
import { AuthProvider } from '@site/src/hooks/useAuth';
import Link from '@docusaurus/Link';
import styles from '@site/src/components/Auth/Profile.module.css';

export default function ProfilePage() {
    const handleSignOut = () => {
        localStorage.removeItem('auth_session');
        localStorage.removeItem('chatkit-thread-id');
        window.location.href = '/signin';
    };

    return (
        <Layout
            title="My Profile"
            description="View and edit your profile"
        >
            <main className={styles.container}>
                <div className={styles.card}>
                    <AuthProvider>
                        <Profile />
                    </AuthProvider>

                    {/* Navigation links */}
                    <div style={{
                        marginTop: '20px',
                        textAlign: 'center',
                        borderTop: '1px solid var(--ifm-color-emphasis-300)',
                        paddingTop: '20px'
                    }}>
                        <Link
                            to="/"
                            style={{
                                color: 'var(--ifm-link-color)',
                                marginRight: '20px'
                            }}
                        >
                            ← Back to Home
                        </Link>
                        <button
                            onClick={handleSignOut}
                            className={styles.logoutButton}
                        >
                            Sign Out
                        </button>
                    </div>
                </div>
            </main>
        </Layout>
    );
}
