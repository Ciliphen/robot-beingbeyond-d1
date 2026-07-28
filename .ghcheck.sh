#!/usr/bin/env bash
echo '--- org memberships (incl. private) ---'
gh api user/memberships/orgs 2>&1 | head -40
echo
echo '--- syswonder org visibility ---'
gh api orgs/syswonder 2>&1 | tr ',' '\n' | grep -E '"login"|members_can_create|default_repository_permission' | head
