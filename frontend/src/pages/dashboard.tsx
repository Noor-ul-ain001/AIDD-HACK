import React, { useEffect, useState } from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import { useAuth } from '../contexts/AuthContext';
import { usePersonalization } from '../contexts/PersonalizationContext';
import { ProgressAPI, Progress } from '../lib/api';
import styles from './dashboard.module.css';

type ModuleProgress = {
  id: string;
  title: string;
  weeks: number;
  completed: number;
  progress: number; // 0-100
  difficulty: string;
};

type Activity = {
  icon: string;
  title: string;
  description: string;
  time: string;
};

export default function DashboardPage(): JSX.Element {
  const { user, isAuthenticated, isLoading } = useAuth();
  const { experienceLevel, learningGoals } = usePersonalization();
  const [greeting, setGreeting] = useState('Welcome back');
  const [progressData, setProgressData] = useState<Progress[]>([]);
  const [isLoadingProgress, setIsLoadingProgress] = useState(true);
  const [isRefreshing, setIsRefreshing] = useState(false);

  useEffect(() => {
    // Set greeting based on time of day
    const hour = new Date().getHours();
    if (hour < 12) setGreeting('Good morning');
    else if (hour < 18) setGreeting('Good afternoon');
    else setGreeting('Good evening');
  }, []);

  // Redirect if not authenticated
  useEffect(() => {
    if (!isLoading && !isAuthenticated) {
      if (typeof window !== 'undefined') {
        window.location.href = '/signin';
      }
    }
  }, [isLoading, isAuthenticated]);

  // Fetch progress data
  useEffect(() => {
    const fetchProgress = async () => {
      if (!isAuthenticated) {
        setIsLoadingProgress(false);
        return;
      }

      console.log('[Dashboard] Fetching progress data...');

      try {
        const response = await ProgressAPI.getAll();
        console.log('[Dashboard] Progress API response:', response);
        if (response.data) {
          console.log('[Dashboard] Progress data received:', response.data);
          setProgressData(response.data);
        }
      } catch (error) {
        console.error('[Dashboard] Error fetching progress:', error);
      } finally {
        setIsLoadingProgress(false);
      }
    };

    fetchProgress();
  }, [isAuthenticated]);

  // Refresh progress when window gains focus (user returns to dashboard)
  useEffect(() => {
    const handleFocus = async () => {
      if (!isAuthenticated) return;

      console.log('[Dashboard] Window focused, refreshing progress...');
      try {
        const response = await ProgressAPI.getAll();
        if (response.data) {
          console.log('[Dashboard] Progress refreshed:', response.data);
          setProgressData(response.data);
        }
      } catch (error) {
        console.error('[Dashboard] Error refreshing progress:', error);
      }
    };

    window.addEventListener('focus', handleFocus);
    return () => window.removeEventListener('focus', handleFocus);
  }, [isAuthenticated]);

  // Manual refresh function
  const handleRefresh = async () => {
    if (!isAuthenticated) return;

    setIsRefreshing(true);
    console.log('[Dashboard] Manual refresh triggered...');

    try {
      const response = await ProgressAPI.getAll();
      if (response.data) {
        console.log('[Dashboard] Progress refreshed manually:', response.data);
        setProgressData(response.data);
      }
    } catch (error) {
      console.error('[Dashboard] Error refreshing progress:', error);
    } finally {
      setIsRefreshing(false);
    }
  };

  // Calculate module progress based on actual data
  const calculateModuleProgress = (moduleId: string, weeks: number): { completed: number; progress: number } => {
    const moduleChapters = progressData.filter(p => p.chapter_id.startsWith(moduleId));
    const completedWeeks = new Set(
      moduleChapters
        .filter(p => p.completion_percentage >= 100)
        .map(p => {
          // Extract week from chapter_id (e.g., "module-1/week-1-introduction/1-1-what-is-ros2")
          const match = p.chapter_id.match(/week-(\d+)/);
          return match ? match[1] : null;
        })
        .filter(Boolean)
    ).size;

    const progress = weeks > 0 ? Math.round((completedWeeks / weeks) * 100) : 0;
    return { completed: completedWeeks, progress };
  };

  const modules: ModuleProgress[] = [
    {
      id: 'module-1',
      title: 'ROS 2 Fundamentals',
      weeks: 4,
      ...calculateModuleProgress('module-1', 4),
      difficulty: 'beginner'
    },
    {
      id: 'module-2',
      title: 'Robot Simulation',
      weeks: 4,
      ...calculateModuleProgress('module-2', 4),
      difficulty: 'intermediate'
    },
    {
      id: 'module-3',
      title: 'NVIDIA Isaac Sim',
      weeks: 4,
      ...calculateModuleProgress('module-3', 4),
      difficulty: 'intermediate'
    },
    {
      id: 'module-4',
      title: 'VLA Models',
      weeks: 4,
      ...calculateModuleProgress('module-4', 4),
      difficulty: 'advanced'
    }
  ];

  // Filter modules based on user's experience level if personalization is on
  const filteredModules = modules.filter(module => {
    if (experienceLevel === 'all') return true;
    if (experienceLevel === 'beginner') return module.difficulty === 'beginner';
    if (experienceLevel === 'intermediate') return ['beginner', 'intermediate'].includes(module.difficulty);
    return true; // advanced sees all
  });

  // Calculate overall progress
  const completedChapters = progressData.filter(p => p.completion_percentage >= 100).length;
  const totalChapters = 16; // Total chapters in the curriculum
  const overallProgress = totalChapters > 0 ? Math.round((completedChapters / totalChapters) * 100) : 0;

  if (isLoading) {
    return (
      <Layout title="Dashboard">
        <main className={styles.main}>
          <div className={styles.container}>
            <div className={styles.loading}>Loading your dashboard...</div>
          </div>
        </main>
      </Layout>
    );
  }

  if (!isAuthenticated) {
    return null; // Will redirect
  }

  const userName = user?.email?.split('@')[0] || 'there';
  const experienceBadge = experienceLevel === 'beginner' ? '🌱' :
                         experienceLevel === 'intermediate' ? '🚀' :
                         experienceLevel === 'advanced' ? '⚡' : '🌐';

  return (
    <Layout title="Dashboard" description="Your Physical AI & Humanoid Robotics learning dashboard">
      <main className={styles.main}>
        <div className={styles.container}>
          {user && !user.email_verified && (
            <div className={styles.emailVerificationBanner}>
              <div className={styles.bannerContent}>
                <span className={styles.bannerIcon}>✉️</span>
                <div className={styles.bannerText}>
                  <p><strong>Email Verification Needed:</strong> Please check your inbox for a verification email. Verify your email to get full access to all features.</p>
                </div>
              </div>
            </div>
          )}
          <header className={styles.header}>
            <h1>{greeting}, {userName}! {experienceBadge}</h1>
            <p>Here's your personalized progress in the Physical AI & Humanoid Robotics curriculum.</p>
            {user?.hardware_profile?.experience_level && (
              <div className={styles.userBadge}>
                <span className={styles.badgeLabel}>Experience Level:</span>
                <span className={styles.badgeValue}>
                  {user.hardware_profile.experience_level.charAt(0).toUpperCase() +
                   user.hardware_profile.experience_level.slice(1)}
                </span>
              </div>
            )}
          </header>

          <section className={styles.overview}>
            <div className={styles.card}>
              <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
                <h2>Overall Progress</h2>
                <button
                  onClick={handleRefresh}
                  disabled={isRefreshing}
                  className={styles.refreshButton}
                  title="Refresh progress data"
                >
                  {isRefreshing ? '🔄 Refreshing...' : '🔄 Refresh'}
                </button>
              </div>
              <div className={styles.progressContainer}>
                <div className={styles.progressBar}>
                  <div
                    className={styles.progressFill}
                    style={{ width: `${overallProgress}%` }}
                  ></div>
                </div>
                <span className={styles.progressText}>{overallProgress}% Complete</span>
              </div>
              <p>{completedChapters} of {totalChapters} chapters completed</p>
            </div>

            <div className={styles.card}>
              <h2>Quick Actions</h2>
              <div className={styles.quickActions}>
                <Link to="/docs/intro" className={styles.actionButton}>
                  Continue Learning
                </Link>
                <Link to="/profile" className={styles.actionButton}>
                  Edit Profile
                </Link>
                <Link to="/chat" className={styles.actionButton}>
                  Ask Assistant
                </Link>
              </div>
            </div>
          </section>

          <section className={styles.modules}>
            <h2>Your Curriculum ({filteredModules.length} modules)</h2>
            <div className={styles.moduleGrid}>
              {filteredModules.map(module => {
                const difficultyBadge = module.difficulty === 'beginner' ? '🌱' :
                                       module.difficulty === 'intermediate' ? '🚀' : '⚡';
                return (
                  <div key={module.id} className={styles.moduleCard}>
                    <div className={styles.moduleHeader}>
                      <h3>{difficultyBadge} {module.title}</h3>
                      <span className={styles.progressText}>{module.progress}%</span>
                    </div>
                    <div className={styles.moduleProgress}>
                      <div
                        className={styles.progressFill}
                        style={{ width: `${module.progress}%` }}
                      ></div>
                    </div>
                    <div className={styles.moduleDetails}>
                      <span>{module.completed} of {module.weeks} weeks completed</span>
                      <Link to={`/docs/${module.id}/week-1-introduction`}>
                        Start Module
                      </Link>
                    </div>
                  </div>
                );
              })}
            </div>
          </section>

          {learningGoals && learningGoals.length > 0 && (
            <section className={styles.goals}>
              <h2>Your Learning Goals</h2>
              <div className={styles.goalsList}>
                {learningGoals.map((goal, index) => (
                  <div key={index} className={styles.goalItem}>
                    <span className={styles.goalIcon}>🎯</span>
                    <span className={styles.goalText}>{goal}</span>
                  </div>
                ))}
              </div>
            </section>
          )}

          <section className={styles.recentActivity}>
            <h2>Getting Started</h2>
            <div className={styles.activityList}>
              <div className={styles.activityItem}>
                <span className={styles.activityIcon}>👋</span>
                <div className={styles.activityContent}>
                  <h4>Welcome to Physical AI & Humanoid Robotics!</h4>
                  <p>Start your learning journey with Module 1: ROS 2 Fundamentals</p>
                  <span className={styles.activityTime}>Just now</span>
                </div>
              </div>
              <div className={styles.activityItem}>
                <span className={styles.activityIcon}>⚙️</span>
                <div className={styles.activityContent}>
                  <h4>Personalization Enabled</h4>
                  <p>Content is filtered for your {experienceLevel} level. Adjust anytime!</p>
                  <span className={styles.activityTime}>Just now</span>
                </div>
              </div>
              {user?.hardware_profile?.robotics_kit && (
                <div className={styles.activityItem}>
                  <span className={styles.activityIcon}>🤖</span>
                  <div className={styles.activityContent}>
                    <h4>Hardware Profile Set</h4>
                    <p>You're using: {user.hardware_profile.robotics_kit}</p>
                    <span className={styles.activityTime}>During setup</span>
                  </div>
                </div>
              )}
            </div>
          </section>
        </div>
      </main>
    </Layout>
  );
}