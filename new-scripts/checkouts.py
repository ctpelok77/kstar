import os
import sys
import logging
import re
import itertools

import tools
from tools import run_command

CHECKOUTS_DIR = os.path.join(tools.SCRIPTS_DIR, 'checkouts')
tools.makedirs(CHECKOUTS_DIR)

ABS_REV_CACHE = {}


class Checkout(object):
    def __init__(self, part, repo, rev, checkout_dir):
        # Directory name of the planner part (translate, preprocess, search)
        self.part = part
        self.repo = repo
        self.rev = str(rev)

        if not os.path.isabs(checkout_dir):
            checkout_dir = os.path.join(CHECKOUTS_DIR, checkout_dir)
        self.checkout_dir = os.path.abspath(checkout_dir)

    def __lt__(self, other):
        return self.name < other.name

    def __eq__(self, other):
        return self.name == other.name

    def __hash__(self):
        return hash(self.name)

    @property
    def name(self):
        """
        Nickname for the checkout that is used for the reports and comparisons.
        """
        if self.rev == 'WORK':
            return 'WORK'
        return os.path.basename(self.checkout_dir)

    def checkout(self):
        raise Exception('Not implemented')

    def compile(self):
        """
        We issue the build_all command unconditionally and let "make" take care
        of checking if something has to be recompiled.
        """
        cwd = os.getcwd()
        os.chdir(self.src_dir)
        try:
            retcode, _, _ = run_command(['./build_all'])
        except OSError:
            logging.error('Changeset %s does not have the build_all script. '
                          'Revision cannot be used by the scripts.' % self.rev)
            sys.exit(1)
        if not retcode == 0:
            logging.error('Build script failed in: %s' % self.src_dir)
            sys.exit(1)
        os.chdir(cwd)

    def get_path(self, *rel_path):
        return os.path.join(self.checkout_dir, *rel_path)

    def get_bin(self, *bin_path):
        """Return the absolute path to one of this part's executables."""
        return os.path.join(self.bin_dir, *bin_path)

    def get_path_dest(self, *rel_path):
        return os.path.join('code-' + self.name, *rel_path)

    def get_bin_dest(self):
        return self.get_path_dest(self.part)

    @property
    def src_dir(self):
        """Returns the path to the global Fast Downward source directory.

        The directory "downward" dir has been renamed to "src", but we still
        want to support older changesets."""
        assert os.path.exists(self.checkout_dir), self.checkout_dir
        src_dir = self.get_path('downward')
        if not os.path.exists(src_dir):
            src_dir = self.get_path('src')
        return src_dir

    @property
    def bin_dir(self):
        return os.path.join(self.src_dir, self.part)

    @property
    def parent_rev(self):
        raise Exception('Not implemented')

    @property
    def shell_name(self):
        return '%s_%s' % (self.part.upper(), self.name)


# ---------- Mercurial --------------------------------------------------------

class HgCheckout(Checkout):
    """
    Base class for the three checkout types (translate, preprocess, search).
    """
    DEFAULT_URL = tools.BASE_DIR
    DEFAULT_REV = 'WORK'

    def __init__(self, part, repo=DEFAULT_URL, rev=DEFAULT_REV, dest=''):
        """
        part: One of translate, preprocess, search
        repo: Path to the hg repository. Can be either local or remote.
        rev:  Changeset. Can be any valid hg revision specifier or "WORK"
        dest: If set this will be the checkout's name. Use this if you need to
              checkout the same revision multiple times and want to alter each
              checkout manually (e.g. for comparing Makefile options).
        """
        if dest and rev == 'WORK':
            logging.error('You cannot have multiple copies of the working '
                          'copy. Please specify a specific revision.')
            sys.exit(1)

        if not os.path.abspath(repo) == HgCheckout.DEFAULT_URL and not dest:
            logging.warning('You should set "dest" explicitly when using a '
                            'remote repo. Otherwise local checkouts might '
                            'overwrite checkouts of the same revision from '
                            'the remote repo.')
            sys.exit(1)

        # Find proper absolute revision
        rev = self.get_abs_rev(repo, rev)

        if rev.upper() == 'WORK':
            checkout_dir = tools.BASE_DIR
        else:
            checkout_dir = dest if dest else rev

        Checkout.__init__(self, part, repo, rev, checkout_dir)
        self.parent = None

    def get_abs_rev(self, repo, rev):
        if str(rev).upper() == 'WORK':
            return 'WORK'
        cmd = ['hg', 'id', '-ir', str(rev).lower(), repo]
        cmd_string = ' '.join(cmd)
        if cmd_string in ABS_REV_CACHE:
            return ABS_REV_CACHE[cmd_string]
        _, abs_rev, _ = run_command(cmd)
        if not abs_rev:
            logging.error('Revision %s not present in repo %s' % (rev, repo))
            sys.exit(1)
        ABS_REV_CACHE[cmd_string] = abs_rev
        return abs_rev

    def checkout(self):
        # We don't need to check out the working copy
        if self.rev == 'WORK':
            return

        path = self.checkout_dir
        if not os.path.exists(path):
            clone = ['hg', 'clone', '-r', self.rev, self.repo, path]
            run_command(clone)
        else:
            logging.info('Checkout "%s" already exists' % path)
            pull = ['hg', 'pull', self.repo]
            run_command(pull, cwd=path)

        update = ['hg', 'update', '-r', self.rev]
        retcode, _, _ = run_command(update, cwd=path)
        if not retcode == 0:
            # Unknown revision
            logging.error('Repo at %s has no revision %s.' % (path, self.rev))
            sys.exit(1)

    @property
    def parent_rev(self):
        if self.parent:
            return self.parent
        rev = self.rev
        if self.rev == 'WORK':
            rev = 'tip'
        cmd = ['hg', 'log', '-r', rev, '--template', '{node|short}']
        _, self.parent, _ = run_command(cmd)
        return self.parent


class Translator(HgCheckout):
    def __init__(self, *args, **kwargs):
        HgCheckout.__init__(self, 'translate', *args, **kwargs)


class Preprocessor(HgCheckout):
    def __init__(self, *args, **kwargs):
        HgCheckout.__init__(self, 'preprocess', *args, **kwargs)


class Planner(HgCheckout):
    def __init__(self, *args, **kwargs):
        HgCheckout.__init__(self, 'search', *args, **kwargs)

# -----------------------------------------------------------------------------


def checkout(combinations):
    """Checks out the code once for each separate checkout directory."""
    # Checkout and compile each revision only once
    for part in sorted(set(itertools.chain(*combinations))):
        part.checkout()


def compile(combinations):
    """Compiles the code."""
    # Checkout and compile each revision only once
    for part in sorted(set(itertools.chain(*combinations))):
        part.compile()
