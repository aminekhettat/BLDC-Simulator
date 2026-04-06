===========================
Branch Protection Setup
===========================

This guide makes the regression and code-quality gates mandatory before merge.

Prerequisites
=============

- GitHub Actions workflow file exists:
  - .github/workflows/regression-gates.yml
  - .github/workflows/code-quality-gates.yml
- Workflows have required jobs:
  - Regression gate job id/check name: regression
  - Quality gate job id/check name: quality

Required Status Check Mapping
=============================

For this repository, the required check to enforce is:

- regression
- quality

Depending on GitHub UI rendering, you may see one of these labels:

- regression
- Regression Gates / regression
- quality
- Code Quality Gates / quality

Select the check that corresponds to the job in the Regression Gates workflow.

UI Setup (Recommended)
======================

1. Open repository Settings.
2. Open Branches.
3. Create or edit a branch protection rule.
4. Use branch pattern:
   - main
   - If your default branch is master, add a second rule for master.
5. Enable Require a pull request before merging.
6. Enable Require status checks to pass before merging.
7. Add required checks:
  - regression
  - quality
8. Optional but recommended:
   - Require branches to be up to date before merging
   - Require conversation resolution before merging
   - Restrict who can push to matching branches

Verification
============

After setup, open a pull request and verify:

1. The Regression Gates workflow runs automatically.
2. The Code Quality Gates workflow runs automatically.
3. Both regression and quality checks appear in PR checks.
4. Merge is blocked until both checks pass.
5. PR template checklist is filled:
  - .github/PULL_REQUEST_TEMPLATE.md

CLI/API Setup (Alternative)
===========================

You can enforce branch protection with GitHub CLI and API.

Example for main branch:

::

    gh api --method PUT /repos/<owner>/<repo>/branches/main/protection \
      --input - << 'JSON'
    {
      "required_status_checks": {
        "strict": true,
        "contexts": ["regression", "quality"]
      },
      "enforce_admins": true,
      "required_pull_request_reviews": {
        "required_approving_review_count": 1
      },
      "restrictions": null,
      "required_linear_history": false,
      "allow_force_pushes": false,
      "allow_deletions": false,
      "block_creations": false,
      "required_conversation_resolution": true,
      "lock_branch": false,
      "allow_fork_syncing": true
    }
    JSON

If your default branch is master, repeat with /branches/master/protection.

Troubleshooting
===============

- If required check cannot be found, run one PR first so GitHub indexes the check name.
- If naming differs in UI, use checks tied to jobs ``regression`` and ``quality`` in their workflows.
- If workflow is skipped due to path filters (none are configured now), status check will not be reported.
- If reviewers miss baseline-change rationale, enforce the checklist in .github/PULL_REQUEST_TEMPLATE.md.
- Enforce baseline regeneration rules from CONTRIBUTING.md during review.
