import os
import sys
import subprocess
import logging
import re
import itertools

import tools

CHECKOUTS_DIR = os.path.join(tools.SCRIPTS_DIR, 'checkouts')
tools.makedirs(CHECKOUTS_DIR)

ABS_REV_CACHE = {}
_sentinel = object()


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
        # We don't need to check out the working copy
        if self.rev == 'WORK':
            return

        # If there's already a checkout, don't checkout again
        path = self.checkout_dir
        if os.path.exists(path):
            logging.info('Checkout "%s" already exists' % path)
        else:
            cmd = self.get_checkout_cmd()
            print cmd
            subprocess.call(cmd, shell=True)
        assert os.path.exists(path), 'Could not checkout to "%s"' % path

    def get_checkout_cmd(self):
        raise Exception('Not implemented')

    def compile(self):
        """
        We issue the build_all command unconditionally and let "make" take care
        of checking if something has to be recompiled.
        """
        cwd = os.getcwd()
        os.chdir(self.get_path('src'))
        subprocess.call(['./build_all'])
        os.chdir(cwd)

    def get_path(self, *rel_path):
        return os.path.join(self.checkout_dir, *rel_path)

    def get_bin(self, *bin_path):
        """Return the absolute path to this part's src directory."""
        return os.path.join(self.bin_dir, *bin_path)

    def get_path_dest(self, *rel_path):
        return os.path.join('code-' + self.name, *rel_path)

    def get_bin_dest(self):
        return self.get_path_dest(self.part)

    @property
    def bin_dir(self):
        assert os.path.exists(self.checkout_dir), self.checkout_dir
        bin_dir = self.get_path('downward', self.part)
        # "downward" dir has been renamed to "src"
        if not os.path.exists(bin_dir):
            bin_dir = self.get_path('src', self.part)
        return bin_dir

    @property
    def parent_rev(self):
        raise Exception('Not implemented')

    @property
    def shell_name(self):
        return '%s_%s' % (self.part.upper(), self.name)


# ---------- Mercurial --------------------------------------------------------

class HgCheckout(Checkout):
    DEFAULT_URL = tools.BASE_DIR
    DEFAULT_REV = 'WORK'

    def __init__(self, part, repo=DEFAULT_URL, rev=DEFAULT_REV, dest=''):
        # Find proper absolute revision
        rev = self.get_abs_rev(repo, rev)

        if rev == 'WORK':
            checkout_dir = tools.BASE_DIR
        else:
            checkout_dir = dest if dest else rev

        Checkout.__init__(self, part, repo, rev, checkout_dir)
        self.parent = None

    def get_abs_rev(self, repo, rev):
        if str(rev).upper() == 'WORK':
            return 'WORK'
        cmd = 'hg id -ir %s %s' % (str(rev).lower(), repo)
        if cmd in ABS_REV_CACHE:
            return ABS_REV_CACHE[cmd]
        abs_rev = tools.run_command(cmd)
        if not abs_rev:
            logging.error('Revision %s not present in repo %s' % (rev, repo))
            sys.exit(1)
        ABS_REV_CACHE[cmd] = abs_rev
        return abs_rev

    def get_checkout_cmd(self):
        cwd = os.getcwd()
        clone = 'hg clone -r %s %s %s' % (self.rev, self.repo,
                                          self.checkout_dir)
        cd_to_repo_dir = 'cd %s' % self.checkout_dir
        update = 'hg update -r %s' % self.rev
        cd_back = 'cd %s' % cwd
        return '; '.join([clone, cd_to_repo_dir, update, cd_back])

    @property
    def parent_rev(self):
        if self.parent:
            return self.parent
        rev = self.rev
        if self.rev == 'WORK':
            rev = 'tip'
        cmd = 'hg log -r %s --template {node|short}' % rev
        self.parent = tools.run_command(cmd)
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


# ---------- Subversion -------------------------------------------------------

class SvnCheckout(Checkout):
    DEFAULT_URL = 'svn+ssh://downward-svn/trunk/downward'
    DEFAULT_REV = 'WORK'

    REV_REGEX = re.compile(r'Revision: (\d+)')

    def __init__(self, part, repo, rev=DEFAULT_REV):
        rev = str(rev)
        name = part + '-' + rev
        rev_abs = self.get_abs_rev(repo, rev)

        if rev == 'WORK':
            logging.error('Comparing SVN working copy is not supported')
            sys.exit(1)

        checkout_dir = part + '-' + rev_abs

        Checkout.__init__(self, part, repo, rev_abs, checkout_dir, name)

    def get_abs_rev(self, repo, rev):
        try:
            # If we have a number string, return it
            int(rev)
            return rev
        except ValueError:
            pass

        if rev.upper() == 'WORK':
            return 'WORK'
        elif rev.upper() == 'HEAD':
            # We want the HEAD revision number
            return self._get_rev(repo)
        else:
            logging.error('Invalid SVN revision specified: %s' % rev)
            sys.exit()

    def _get_rev(self, repo):
        """
        Returns the revision for the given repo. If the repo is a remote URL,
        the HEAD revision is returned. If the repo is a local path, the
        working copy's version is returned.
        """
        env = {'LANG': 'C'}
        cmd = 'svn info %s' % repo
        if cmd in ABS_REV_CACHE:
            return ABS_REV_CACHE[cmd]
        output = tools.run_command(cmd, env=env)
        match = self.REV_REGEX.search(output)
        if not match:
            logging.error('Unable to get HEAD revision number')
            sys.exit()
        rev_number = match.group(1)
        ABS_REV_CACHE[cmd] = rev_number
        return rev_number

    def get_checkout_cmd(self):
        return 'svn co %s@%s %s' % (self.repo, self.rev, self.checkout_dir)

    def compile(self):
        """
        We need to compile the code if the executable does not exist.
        Additionally we want to compile it, when we run an experiment with
        the working copy to make sure the executable is based on the latest
        version of the code. Obviously we don't need no compile the
        translator code.
        """
        if self.part == 'translate':
            return

        if self.rev == 'WORK' or self._get_executable(default=None) is None:
            cwd = os.getcwd()
            os.chdir(self.bin_dir)
            subprocess.call(['make'])
            os.chdir(cwd)

    @property
    def parent_rev(self):
        return self._get_rev(self.checkout_dir)

    @property
    def bin_dir(self):
        # checkout_dir is bin_dir for SVN
        assert os.path.exists(self.checkout_dir)
        return self.checkout_dir


class TranslatorSvnCheckout(SvnCheckout):
    DEFAULT_URL = 'svn+ssh://downward-svn/trunk/downward/translate'

    def __init__(self, repo=DEFAULT_URL, rev=SvnCheckout.DEFAULT_REV):
        SvnCheckout.__init__(self, 'translate', repo, rev)


class PreprocessorSvnCheckout(SvnCheckout):
    DEFAULT_URL = 'svn+ssh://downward-svn/trunk/downward/preprocess'

    def __init__(self, repo=DEFAULT_URL, rev=SvnCheckout.DEFAULT_REV):
        SvnCheckout.__init__(self, 'preprocess', repo, rev)


class PlannerSvnCheckout(SvnCheckout):
    DEFAULT_URL = 'svn+ssh://downward-svn/trunk/downward/search'

    def __init__(self, repo=DEFAULT_URL, rev=SvnCheckout.DEFAULT_REV):
        SvnCheckout.__init__(self, 'search', repo, rev)

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
