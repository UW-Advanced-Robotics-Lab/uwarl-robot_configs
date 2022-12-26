#!/usr/bin/env bash

function markdown-toc(){
    FILE=${1:?No file was specified as first argument}
    
    declare -a TOC
    CODE_BLOCK=0
    CODE_BLOCK_REGEX='.*```'
    CODE_BLOCK_REGEX_PAIR='.*```.*```'
    HEADING_REGEX='\#{1,}'
    
    while read -r LINE; do
        # Treat code blocks
        if [[ "${LINE}" =~ $CODE_BLOCK_REGEX ]]; then
            # Ignore things until we see code block ending
            CODE_BLOCK=$((CODE_BLOCK + 1))
            # Skip in-line code block
            if [[ "${LINE}" =~ $CODE_BLOCK_REGEX_PAIR ]]; then
                echo "inline code block"
                CODE_BLOCK=0
            fi
            if [[ "${CODE_BLOCK}" -eq 2 ]]; then
                # We hit the closing code block
                CODE_BLOCK=0
            fi
            continue
        fi
    
        # Treat normal line
        if [[ "${CODE_BLOCK}" == 0 ]]; then
            # If we see heading, we save it to ToC map
            if [[ "${LINE}" =~ ${HEADING_REGEX} ]]; then
                TOC+=("${LINE}")
            fi
        fi
    done < <(grep -v '\n# Table of Contents' "${FILE}")
    
    TOC_TEXT="<toc>\n\n"
    TOC_TEXT+="# Table of Contents\n"
    TOC_TEXT+="[*Last generated: $(date)*]\n"
    for LINE in "${TOC[@]}"; do
        LINK=${LINE}
        # Detect markdown links in heading and remove link part from them
        if grep -qE "\[.*\]\(.*\)" <<< "${LINK}"; then
            LINK=$(sed 's/\(\]\)\((.*)\)/\1/' <<< "${LINK}")
        fi
        # Special characters (besides '-') in page links in markdown
        # are deleted and spaces are converted to dashes
        LINK=$(tr -dc "[:alnum:] _-" <<< "${LINK}")
        LINK=${LINK/ /}
        LINK=${LINK// /-}
        # LINK=${LINK,,}
        LINK=$(tr -s "-" <<< "${LINK}")
    
        # Print in format [Very Special Heading](#very-special-heading)
        case "${LINE}" in
            '#####'*)
                TOC_TEXT+="        - [${LINE#\#* }](#${LINK})\n"
                ;;
            '####'*)
                TOC_TEXT+="      - [${LINE#\#* }](#${LINK})\n"
                ;;
            '###'*)
                TOC_TEXT+="    - [${LINE#\#* }](#${LINK})\n"
                ;;
            '##'*)
                TOC_TEXT+="  - [${LINE#\#* }](#${LINK})\n"
                ;;
            '#'*)
                TOC_TEXT+="- [**${LINE#\#* }**](#${LINK})\n"
                ;;
        esac

    done
    TOC_TEXT+="\n\n</toc>\n"
    
    { echo -en "${TOC_TEXT}"; cat $FILE; } > $FILE.new
    mv $FILE{.new,}
    echo "    - [x] Generated table of contents"
}

function markdown_toc(){
    file=$1
    if [[ $file = "docs/_Sidebar.md" || $file = "docs/_Footer.md" || $file = "docs/Home.md" ]]; then
        echo "    - [!] Skip $file"
        continue # skip particular md files
    else
        if [[ $(<$file) = *"<toc>"*"</toc>"* ]]; then
        # if [[ $(<$file) = *"# Table of Contents"* ]]; then
            echo "    - [!] Already has a ToC, deleting the old ToC ..."
            sed '/<toc>/,/<\/toc>/d' $file > $file.new
            mv $file{.new,}
            markdown-toc $file
            continue # skip if already has a toc
        else
            markdown-toc $file
        fi
    fi
}

function markdown_toc_directory(){
    # enter main directories:
    cd "$(dirname "$0")"/..
    
    # markdown batch process:
    if [[ $1 = "-a" ]]; then
        # process all documents
        LIST_OF_MD=(docs/*.md)
    else
        # process only '.md' files
        LIST_OF_MD="$(git status --porcelain -- 'docs/*.md' | sed s/^...//)" 
        LIST_OF_MD=($LIST_OF_MD) # convert to array
    fi 
    
    i=0
    for file in "${LIST_OF_MD[@]}"; do
        i=$(( i + 1 ))
        echo "> [$i] - editing toc @ $file"
        markdown_toc $file
    done
}
